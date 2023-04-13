/*
* Copyright (c) 2020, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "tflite_posenet.h"

namespace mle {

static char PartNames[TotalKeypointNum][MaxKeypointNameLen] =
{
  "nose", "leftEye", "rightEye", "leftEar", "rightEar", "leftShoulder",
  "rightShoulder", "leftElbow", "rightElbow", "leftWrist", "rightWrist",
  "leftHip", "rightHip", "leftKnee", "rightKnee", "leftAnkle", "rightAnkle"
};

static char PoseChain[32][MaxKeypointNameLen] =
{
  "nose", "leftEye", "leftEye", "leftEar", "nose", "rightEye", "rightEye",
  "rightEar", "nose", "leftShoulder","leftShoulder", "leftElbow",
  "leftElbow", "leftWrist", "leftShoulder", "leftHip", "leftHip", "leftKnee",
  "leftKnee", "leftAnkle", "nose", "rightShoulder", "rightShoulder",
  "rightElbow", "rightElbow", "rightWrist", "rightShoulder", "rightHip",
  "rightHip", "rightKnee", "rightKnee", "rightAnkle"
};

  //TODO query this information from model
  static const uint32_t       kNumOutputs       = 3;
  static const uint32_t       kOutBytesPerPixel = 4;
  static const uint32_t       kHeatMapSize      = 41 * 31 * 17;
  static const uint32_t       kOffsetSize       = 41 * 31 * 34;
  static const uint32_t       kDisplacementSize = 41 * 31 * 64;

  static const float          output_0_value = 0.0470588244497776;
  static const float          output_1_value = 0.3921568691730499;
  static const float          output_2_value = 1.3875764608383179;

TFLPoseNet::TFLPoseNet(MLConfig &config) : TFLBase(config) {
  pRawHeatmaps = nullptr;
  pRawOffsets = nullptr;
  pRawDisplacements = nullptr;
  need_labels_ = false;
  pose_count_ = 0;
}
TFLPoseNet::~TFLPoseNet() {}

int32_t TFLPoseNet::AllocateInternalBuffers() {
  int32_t ret = MLE_OK;
  posix_memalign(reinterpret_cast<void**>(&buffers_.scale_buf), 128,
                                  ((scale_width_ * scale_height_ * 3) / 2));
  posix_memalign(reinterpret_cast<void**>(&buffers_.rgb_buf), 128,
                                  ((scale_width_ * scale_height_ * 3)));
  // Allocate buffers for dequantization
  posix_memalign(reinterpret_cast<void**>(&pRawHeatmaps), 128,
                                          kHeatMapSize * kOutBytesPerPixel);
  posix_memalign(reinterpret_cast<void**>(&pRawOffsets), 128,
                                          kOffsetSize * kOutBytesPerPixel);
  posix_memalign(reinterpret_cast<void**>(&pRawDisplacements), 128,
                                        kDisplacementSize * kOutBytesPerPixel);

  if (nullptr == buffers_.scale_buf ||
      nullptr == buffers_.rgb_buf ||
      nullptr == pRawHeatmaps ||
      nullptr == pRawOffsets ||
      nullptr == pRawDisplacements) {
    MLE_LOGE("%s: Buffer allocation failed", __func__);
    ret = MLE_FAIL;
  }
  return ret;
}

void TFLPoseNet::FreeInternalBuffers() {
  if (nullptr != buffers_.scale_buf) {
    free(buffers_.scale_buf);
    buffers_.scale_buf = nullptr;
  }
  if (nullptr != buffers_.rgb_buf) {
    free(buffers_.rgb_buf);
    buffers_.rgb_buf = nullptr;
  }
  if (nullptr != pRawHeatmaps) {
    free(pRawHeatmaps);
    pRawHeatmaps = nullptr;
  }
  if (nullptr != pRawOffsets) {
    free(pRawOffsets);
    pRawOffsets = nullptr;
  }
  if (nullptr != pRawDisplacements) {
    free(pRawDisplacements);
    pRawDisplacements = nullptr;
  }
}

int32_t TFLPoseNet::PostProcess(GstBuffer* buffer) {
  MLE_LOGI("%s Enter", __func__);

  PoseResult        instancePoseResults;
  Part              scoredParts[PoseMaxNumScoredParts];
  PartId            partIds[TotalKeypointNum];
  ParentChildTurple parentChildTurples[TotalKeypointNum - 1];

  // Get outputs from model
  uint8_t* pRawHeatmaps_temp =
      tflite_params_.interpreter->typed_output_tensor<uint8_t>(0);
  uint8_t* pRawOffsets_temp =
      tflite_params_.interpreter->typed_output_tensor<uint8_t>(1);
  uint8_t* pRawDisplacements_temp =
      tflite_params_.interpreter->typed_output_tensor<uint8_t>(2);

  // Dequantization of model outputs is needed,
  // because the postprocessing operates on float values
  for (uint32_t i = 0; i < kHeatMapSize; i++) {
    pRawHeatmaps[i] =
        static_cast<float>(pRawHeatmaps_temp[i] - 128) * output_0_value;
  }
  for (uint32_t i = 0; i < kOffsetSize; i++) {
    pRawOffsets[i] =
        static_cast<float>(pRawOffsets_temp[i] - 128) * output_1_value;
  }

  for (uint32_t i = 0; i < kDisplacementSize; i++) {
    pRawDisplacements[i] =
        static_cast<float>(pRawDisplacements_temp[i] - 117) * output_2_value;
  }

  // Change the following values as per your application requirements
  pose_pp_config_.outputStride          = 16;
  pose_pp_config_.maxPoseDetections     = PoseMaxNumDetect;
  pose_pp_config_.minPoseScore          = 0.10f;
  pose_pp_config_.heatmapScoreThreshold = 0.35f;
  pose_pp_config_.nmsRadius             = 20;
  pose_pp_config_.featureHeight         = 31;
  pose_pp_config_.featureWidth          = 41;
  pose_pp_config_.numKeypoint           = 17;
  pose_pp_config_.localMaximumRadius    = 1;

  int squaredNmsRadius             = pow(pose_pp_config_.nmsRadius, 2);
  float candidatePoseInstanceScore = 0.0f;

  int partsCount = SelectKeypointWithScore(&pose_pp_config_, pRawHeatmaps, &scoredParts[0]);

  pose_count_ = 0;

  for (int i = 0; i < PoseMaxNumDetect; i++)
  {
      pose_results_[i].poseScore = 0.0f;
      for (int j = 0; j < pose_pp_config_.numKeypoint; j++)
      {
          pose_results_[i].keypointScore[j]         = 0.0f;
          pose_results_[i].keypointCoord[j * 2]     = 0.0f;
          pose_results_[i].keypointCoord[j * 2 + 1] = 0.0f;
      }
  }
  // Sort selected keypoints according to heatmap scores
  qsort(scoredParts, partsCount, sizeof(Part), SortPartScore);

  int pRawOffsetsHalfSize = pose_pp_config_.featureHeight *
      pose_pp_config_.featureWidth * TotalKeypointNum * 2;
  int pRawDisplacementsHalfSize =
      pose_pp_config_.featureHeight * pose_pp_config_.featureWidth *
      (TotalKeypointNum - 1) * 2;
  float* pOffsets =
      static_cast<float*>(malloc(pRawOffsetsHalfSize * sizeof(float)));
  float* pDisplacementsBwd =
      static_cast<float*>(malloc(pRawDisplacementsHalfSize * sizeof(float)));
  float* pDisplacementsFwd =
      static_cast<float*>(malloc(pRawDisplacementsHalfSize * sizeof(float)));

  if ((NULL == pOffsets) || (NULL == pDisplacementsBwd) || (NULL == pDisplacementsFwd))
  {
      MLE_LOGE("Couldn't allocate buffer for PoseNet raw model short-range and mid-range offset outputs \n");
      exit(0);
  }

  // Reshape short-range offsets, mid-range displacements (backward), and mid-range displacements (forward)
  ReshapeLastTwoDimensions(&pose_pp_config_, pRawOffsets, pOffsets);
  ReshapeDisplacements(&pose_pp_config_, pRawDisplacements, pDisplacementsBwd, pDisplacementsFwd);

  // Generate human keypoint/part graph information
  int partNameLen = 20;
  GeneratePartIds(PartNames[0], &partIds[0], pose_pp_config_.numKeypoint, partNameLen);
  GenerateParentChildTuples(PoseChain[0], &parentChildTurples[0], &partIds[0], pose_pp_config_.numKeypoint, partNameLen);

  // Search adjacent, connected keypoints and propagate pose infomration for each selected keypoint (i.e. root/seed)
  for (int i = 0; i < partsCount; i++)
  {
      float  rootScore = scoredParts[i].partScore;
      int       rootId = scoredParts[i].keypointId;
      int rootCoord[2] = { scoredParts[i].coord[0], scoredParts[i].coord[1] };

      int tmpIdx = rootCoord[0] * pose_pp_config_.featureWidth * pose_pp_config_.numKeypoint * 2 +
          rootCoord[1] * pose_pp_config_.numKeypoint * 2 + rootId * 2;
      float rootImageCoords[2] = { float(rootCoord[0]) * float(pose_pp_config_.outputStride) + pOffsets[tmpIdx],
                                    float(rootCoord[1]) * float(pose_pp_config_.outputStride) + pOffsets[tmpIdx + 1] };

      // Check NMS for the current keypoint root/seed by comparing its location with those of detected poses
      if (DoNMSPose(&pose_results_[0], pose_count_, rootId, squaredNmsRadius, rootImageCoords, pose_pp_config_.numKeypoint))
          continue;

      // Single-pose detection by starting from the current keypoint root/seed and searching adjacent keypoints
      DecodePose(rootScore,
                  rootId,
                  rootImageCoords,
                  pRawHeatmaps,
                  pOffsets,
                  pDisplacementsBwd,
                  pDisplacementsFwd,
                  &pose_pp_config_,
                  &instancePoseResults,
                  &parentChildTurples[0]);

      // Pose score calculation for a single pose instance
      candidatePoseInstanceScore = CalculatePoseInstanceScore(&pose_results_[0],
                                                              pose_count_,
                                                              squaredNmsRadius,
                                                              &instancePoseResults,
                                                              &pose_pp_config_);

      if (candidatePoseInstanceScore > pose_pp_config_.minPoseScore)
      {
          pose_results_[pose_count_].poseScore = candidatePoseInstanceScore;
          for (int j = 0; j < pose_pp_config_.numKeypoint; j++)
          {
              pose_results_[pose_count_].keypointScore[j]         = instancePoseResults.keypointScore[j];
              pose_results_[pose_count_].keypointCoord[j * 2]     = instancePoseResults.keypointCoord[j * 2];
              pose_results_[pose_count_].keypointCoord[j * 2 + 1] = instancePoseResults.keypointCoord[j * 2 + 1];
          }
          pose_count_ += 1;
      }

      if (pose_count_ >= PoseMaxNumDetect)
          break;
  }

  if (buffer) {
    scale_back_x_ = source_params_.width / scale_width_;
    scale_back_y_ = source_params_.height / scale_height_;
    // Display final results for pose estimation
    MLE_LOGD("\nPose Estimation Results (coordinates: [y(row), x(col)]):\n\n");
    for (int i = 0; i < pose_count_; i++)
    {
        MLE_LOGD("Pose #%d,  score = %.4f\n", i, pose_results_[i].poseScore);
        GstMLPoseNetMeta *meta = gst_buffer_add_posenet_meta (buffer);
        if (!meta) {
            MLE_LOGD("%s: Failed to add metadata!", __func__);
            free(pOffsets);
            free(pDisplacementsBwd);
            free(pDisplacementsFwd);
            return MLE_FAIL;
        }
        meta->score = pose_results_[i].poseScore;

        for (int j = 0; j < pose_pp_config_.numKeypoint; j++)
        {
            MLE_LOGD("Keypoint ID:%2d score = %.4f,  coords = [%.2f, %.2f] \n",
                j,
                pose_results_[i].keypointScore[j],
                pose_results_[i].keypointCoord[j * 2],
                pose_results_[i].keypointCoord[j * 2 + 1]);

            meta->points[j].score = pose_results_[i].keypointScore[j];
            meta->points[j].x = (int32_t)
                (pose_results_[i].keypointCoord[j * 2 + 1] * scale_back_x_);
            meta->points[j].y = (int32_t)
                (pose_results_[i].keypointCoord[j * 2] * scale_back_y_);
        }
    }
  }

  free(pOffsets);
  free(pDisplacementsBwd);
  free(pDisplacementsFwd);

  MLE_LOGI("%s Exit", __func__);
  return MLE_OK;
}

int TFLPoseNet::ScoreSort(
    const void* pVal1,
    const void* pVal2)
{
    Score* pScore1 = (Score*)pVal1;
    Score* pScore2 = (Score*)pVal2;

    if (pScore1->val > pScore2->val)
    {
        return -1;
    }

    if (pScore1->val < pScore2->val)
    {
        return 1;
    }

    return 0;
}

int TFLPoseNet::SortPartScore(
    const void* pVal1,
    const void* pVal2)
{
    const Part* pScore1 = static_cast<const Part*>(pVal1);
    const Part* pScore2 = static_cast<const Part*>(pVal2);

    if (pScore1->partScore > pScore2->partScore)
    {
        return -1;
    }
    if (pScore1->partScore < pScore2->partScore)
    {
        return 1;
    }

    return 0;
}

void TFLPoseNet::SortTopN(
    Score* pScores,
    int numClasses,
    int topNum,
    Score* pTopNScores)
{
    for (int i = 0; i < topNum; i++)
    {
        pTopNScores[i].val   = -100.0;
        pTopNScores[i].index = 0;
    }

    for (int i = 0; i < numClasses; i++)
    {
        for (int rank = 0; rank < topNum; rank++)
        {
            if (ScoreSort(&pScores[i], &pTopNScores[rank]) < 0)
            {
                for(int j = topNum -1; j > rank ; j--)
                {
                    pTopNScores[j] = pTopNScores[j-1];
                }
                pTopNScores[rank] = pScores[i];
                break;
            }
        }
    }
}

void TFLPoseNet::ListTopN(
    float* pClassPred,
    int    topNum,
    char   labels[][128],
    FILE*  pMetaFile)
{
    // Change the following values as per your application requirements
    int numClasses = 1024;

    Score* pScores = (Score*)malloc(numClasses * sizeof(Score));
    Score* pTopN   = (Score*)malloc(topNum * sizeof(Score));
    Score* pSorted = NULL;
    if ((NULL != pScores) && (NULL != pTopN))
    {
        for (int i = 0; i < numClasses; i++)
        {
            pScores[i].val   = pClassPred[i];
            pScores[i].index = i;
        }

        if (NULL != pMetaFile)
        {
            SortTopN(pScores, numClasses, topNum, pTopN);
            pSorted = pTopN;
        }
        else
        {
            qsort(pScores, numClasses, sizeof(Score), ScoreSort);
            pSorted = pScores;
        }

        for (int i = 0; i < topNum; i++)
        {
            MLE_LOGD("Rank:%d, Score:%f, Class:%s, ClassIdx:%d \n",
                   i,
                   pSorted[i].val,
                   labels[pSorted[i].index],
                   pSorted[i].index);

            if (NULL != pMetaFile)
            {
                fprintf(pMetaFile, "{ \"id\":%u, \"display_name\":\"%s\", \"type\":%d, \"confidence\":%f }",
                         i,
                         labels[pSorted[i].index],
                         0,
                         pSorted[i].val);

                if ((topNum - 1) != i)
                {
                    fprintf(pMetaFile, ", ");
                }
            }
        }

        free(pScores);
        free(pTopN);
    }
    else
    {
        MLE_LOGE("%s: Couldn't allocate memory!", __FUNCTION__);
    }
}

void TFLPoseNet::CalculateLabel(
        int*   pLabelMap,
        char   labels[][128],
        int    inHeight,
        int    inWidth)
{
        // Change the following values as per your application requirements
        int num[21] = { 0 };

        for (int j = 0; j < inHeight * inWidth; j++)
        {
            num[pLabelMap[j]] += 1;
        }
        for (int j = 0; j < 21; j++)
        {
            if (num[j] != 0)
            {
                MLE_LOGE("This image contain %d pixels of label \"%s\" \n", num[j], labels[j]);
            }
        }
}

void TFLPoseNet::DoHeatmapNormalize(
    float* pRawScores,
    int    rawScoresSize)
{
    for (int i = 0; i < rawScoresSize; i++)
    {
        pRawScores[i] = 1.0f / (1.0f + exp(-pRawScores[i]));
    }
}

void TFLPoseNet::FindMaximumFilterForVector(
    PosePPConfig* pPosePPConfig,
    float*        pKpScoresRow,
    float*        pMaxValsRow)
{
    float*   pOld = pKpScoresRow;
    float*   pNew = pMaxValsRow;
    int    maxIdx = 0;
    int tmpRadius = pPosePPConfig->localMaximumRadius;
    int blockSize = tmpRadius * 2 + 1;
    int  tmpWidth = pPosePPConfig->featureWidth;
    float  maxVal = pOld[0];

    // Initialization on left-boundary values
    for (int tmpIdx = 1; tmpIdx < blockSize; tmpIdx++)
    {
        float curVal = pOld[tmpIdx];
        if (maxVal <= curVal)
        {
            maxVal = curVal;
            maxIdx = tmpIdx;
        }
    }
    for (int i = 0; i <= tmpRadius; i++)
    {
        pNew[i] = maxVal;
    }

    // Process non-boundary values
    for (int tmpIdx = tmpRadius + 1; tmpIdx < tmpWidth - tmpRadius; tmpIdx++)
    {
        if (maxIdx >= (tmpIdx - tmpRadius))
        {
            int nextBlockFirstIndex = tmpIdx + tmpRadius;
            if (pOld[maxIdx] < pOld[nextBlockFirstIndex])
            {
                maxIdx = nextBlockFirstIndex;
                maxVal = pOld[nextBlockFirstIndex];
            }
        }
        else
        {
            maxIdx       = tmpIdx - tmpRadius;
            maxVal       = pOld[maxIdx];
            int blockEnd = maxIdx + blockSize;

            for (int tmpBIdx = maxIdx; tmpBIdx < blockEnd; tmpBIdx++)
            {
                float tmpVal = pOld[tmpBIdx];
                if (maxVal <= tmpVal)
                {
                    maxIdx = tmpBIdx;
                    maxVal = tmpVal;
                }
            }
        }
        pNew[tmpIdx] = maxVal;
    }

    // Process right-boundary values
    for (int i = tmpWidth - tmpRadius; i < tmpWidth; i++)
    {
        pNew[i] = maxVal;
    }
}

void TFLPoseNet::FindMaximumFilterForMatrix(
    PosePPConfig* pPosePPConfig,
    float*        pKpScores,
    float*        pFilteredKpScores)
{
    int     tmpHeight = pPosePPConfig->featureHeight;
    int      tmpWidth = pPosePPConfig->featureWidth;
    float  tmpMatrix[PoseFeatureMapSize] = { 0.0f };
    float tmpMatrixT[PoseFeatureMapSize] = { 0.0f };

    // Maximum filtering on each row of one matrix
    for (int row = 0; row < tmpHeight; row++)
    {
        float* pOldRowFirst = pKpScores + row * tmpWidth;
        float* pNewRowFirst = tmpMatrix + row * tmpWidth;
        FindMaximumFilterForVector(pPosePPConfig, pOldRowFirst, pNewRowFirst);
    }

    // Transpose a matrix
    for (int row = 0; row < tmpWidth; row++)
    {
        for (int col = 0; col < tmpHeight; col++)
        {
            tmpMatrixT[row * tmpHeight + col] = tmpMatrix[col * tmpWidth + row];
        }
    }

    // Maximum filtering on each column of a matrix (i.e. each row of the transposed matrix)
    for (int col = 0; col < tmpWidth; col++)
    {
        float* pOldRowFirst = tmpMatrixT + col * tmpHeight;
        float* pNewRowFirst = tmpMatrix + col * tmpHeight;
        FindMaximumFilterForVector(pPosePPConfig, pOldRowFirst, pNewRowFirst);
    }

    // Transpose a matrix back
    for (int row = 0; row < tmpHeight; row++)
    {
        for (int col = 0; col < tmpWidth; col++)
        {
            pFilteredKpScores[row * tmpWidth + col] = tmpMatrix[col * tmpHeight + row];
        }
    }
}

int TFLPoseNet::SelectKeypointWithScore(
    PosePPConfig* pPosePPConfig,
    float*        pRawScores,
    Part*         pParts)
{
    float heatmapScoreThreshold = pPosePPConfig->heatmapScoreThreshold;
    // int      localMaximumRadius = pPosePPConfig->localMaximumRadius;
    int            numKeypoints = pPosePPConfig->numKeypoint;
    int              partsCount = 0;

    float         kpScores[PoseFeatureMapSize] = { 0.0f };
    float filteredKpScores[PoseFeatureMapSize] = { 0.0f };
    // Heatmap normalization to range [0, 1] via a Sigmoid function
    int rawScoresSize = pPosePPConfig->featureHeight * pPosePPConfig->featureWidth * pPosePPConfig->numKeypoint;
    DoHeatmapNormalize(pRawScores, rawScoresSize);
    // Iterate over keypoints and apply local maximum filtering on the feature map/array corresponding to each keypoint
    for (int keypointId = 0; keypointId < numKeypoints; keypointId++)
    {

        // Apply thresholding over raw heatmap values to remove keypoints with low heatmap values
        for (int i = 0; i < pPosePPConfig->featureHeight; i++)
        {
            for (int j = 0; j < pPosePPConfig->featureWidth; j++)
            {
                float tempVal = 0.0f;
                kpScores[i * pPosePPConfig->featureWidth + j]         = 0.0f;
                filteredKpScores[i * pPosePPConfig->featureWidth + j] = 0.0f;
                tempVal = pRawScores[i * pPosePPConfig->featureWidth * numKeypoints + j * numKeypoints + keypointId];
                tempVal = (tempVal > heatmapScoreThreshold) ? tempVal : 0.0f;
                kpScores[i * pPosePPConfig->featureWidth + j] = tempVal;
            }
        }

        // Apply maximum filtering on the heatmap corresponding to each keypoint
        FindMaximumFilterForMatrix(pPosePPConfig, kpScores, filteredKpScores);

        for (int i = 0; i < pPosePPConfig->featureHeight; i++)
        {
            for (int j = 0; j < pPosePPConfig->featureWidth; j++)
            {
                int tmpIdx = i * pPosePPConfig->featureWidth + j;
                if ((kpScores[tmpIdx] == filteredKpScores[tmpIdx]) && (kpScores[tmpIdx] > 0))
                {
                    pParts[partsCount].coord[0]   = i;
                    pParts[partsCount].coord[1]   = j;
                    pParts[partsCount].partScore  =
                        pRawScores[i * pPosePPConfig->featureWidth * numKeypoints + j * numKeypoints + keypointId];
                    pParts[partsCount].keypointId = keypointId;
                    partsCount++;
                }
            }
        }
    }
    return partsCount;
}

void TFLPoseNet::ReshapeLastTwoDimensions(
    PosePPConfig* pPosePPConfig,
    float*        pRawOffsets,
    float*        pReshapedOffsets)
{
    int tmpIdxRawY  = 0;
    int tmpIdxNewY  = 0;
    int tmpIdxRawX  = 0;
    int tmpIdxNewX  = 0;
    int         fH  = pPosePPConfig->featureHeight;
    int         fW  = pPosePPConfig->featureWidth;
    int numKeypoint = pPosePPConfig->numKeypoint;

    // New shape: [fH, fW, numKeypoint, 2], old shape: [fH, fW, 2, numKeypoint]
    for (int i = 0; i < fH; i++)
    {
        for (int j = 0; j < fW; j++)
        {
            for (int k = 0; k < numKeypoint; k++)
            {
                tmpIdxNewY = i * fW * numKeypoint * 2 + j * numKeypoint * 2 + k * 2;
                tmpIdxNewX = tmpIdxNewY + 1;
                tmpIdxRawY = i * fW * numKeypoint * 2 + j * numKeypoint * 2 + k;
                tmpIdxRawX = tmpIdxRawY + numKeypoint;
                pReshapedOffsets[tmpIdxNewY] = pRawOffsets[tmpIdxRawY];
                pReshapedOffsets[tmpIdxNewX] = pRawOffsets[tmpIdxRawX];
            }
        }
    }
}

void TFLPoseNet::ReshapeDisplacements(
    PosePPConfig* pPosePPConfig,
    float*        pRawDisplacements,
    float*        pReshapedDisplacementsBwd,
    float*        pReshapedDisplacementsFwd)
{
    int tmpIdxBwdRawY = 0;
    int tmpIdxBwdRawX = 0;
    int tmpIdxFwdRawY = 0;
    int tmpIdxFwdRawX = 0;
    int    tmpIdxNewY = 0;
    int    tmpIdxNewX = 0;
    int            fH = pPosePPConfig->featureHeight;
    int            fW = pPosePPConfig->featureWidth;
    int       numEdge = pPosePPConfig->numKeypoint - 1;

    // New shape: [fH, fW, numEdge, 2] (one tensor for BWD and one tensor for FWD), old shape: [fH, fW, 4, numEdge]
    for (int i = 0; i < fH; i++)
    {
        for (int j = 0; j < fW; j++)
        {
            for (int k = 0; k < numEdge; k++)
            {
                tmpIdxFwdRawY = i * fW * numEdge * 4 + j * numEdge * 4 + k;
                tmpIdxFwdRawX = tmpIdxFwdRawY + numEdge;
                tmpIdxBwdRawY = i * fW * numEdge * 4 + j * numEdge * 4 + 2 * numEdge + k;
                tmpIdxBwdRawX = tmpIdxBwdRawY + numEdge;
                tmpIdxNewY    = i * fW * numEdge * 2 + j * numEdge * 2 + k * 2;
                tmpIdxNewX    = tmpIdxNewY + 1;
                pReshapedDisplacementsBwd[tmpIdxNewY] = pRawDisplacements[tmpIdxBwdRawY];
                pReshapedDisplacementsBwd[tmpIdxNewX] = pRawDisplacements[tmpIdxBwdRawX];
                pReshapedDisplacementsFwd[tmpIdxNewY] = pRawDisplacements[tmpIdxFwdRawY];
                pReshapedDisplacementsFwd[tmpIdxNewX] = pRawDisplacements[tmpIdxFwdRawX];
            }
        }
    }
}

int TFLPoseNet::DoNMSPose(
    PoseResult* pPoseResults,
    int         poseCount,
    int         rootId,
    int         squaredNmsRadius,
    float       curPoint[2],
    int         numKeypoints)
{
    MLE_UNUSED(numKeypoints);
    float tmpval = 0.0f;
    for (int i = 0; i < poseCount; i++)
    {
        tmpval = pow(pow(curPoint[0] - pPoseResults[i].keypointCoord[rootId * 2], 2) +
            pow(curPoint[1] - pPoseResults[i].keypointCoord[rootId * 2 + 1], 2), 2);
        if (tmpval < squaredNmsRadius)
            return 1;
    }

    return 0;
}

void TFLPoseNet::GeneratePartIds(
    char*   PartNames,
    PartId* pPartIds,
    int     numKeypoint,
    int     partNameLen)
{
    for (int i = 0; i < numKeypoint; i++)
    {
        pPartIds[i].pId = i;
        g_strlcpy(pPartIds[i].pName, PartNames + i * partNameLen,
          sizeof(pPartIds[i].pName));
    }
}

void TFLPoseNet::GenerateParentChildTuples(
    char*              PoseChain,
    ParentChildTurple* pParentChildTurples,
    PartId*            pPartIds,
    int                numKeypoint,
    int                partNameLen)
{
    int numEdges = numKeypoint - 1;
    for (int i = 0; i < numEdges; i++)
    {
        char parent[MaxKeypointNameLen] = { 0 };
        char  child[MaxKeypointNameLen] = { 0 };
        g_strlcpy(parent, PoseChain + i * partNameLen * 2, sizeof(parent));
        g_strlcpy(child, PoseChain + i * partNameLen * 2 + partNameLen,
            sizeof(child));

        for (int j = 0; j < numKeypoint; j++)
        {
            if ((!strcmp(parent, pPartIds[j].pName))) {
                pParentChildTurples[i].parent = pPartIds[j].pId;
            }
        }
        for (int j = 0; j < numKeypoint; j++)
        {
            if ((!strcmp(child, pPartIds[j].pName)))
                pParentChildTurples[i].child = pPartIds[j].pId;
        }
    }
}

void TFLPoseNet::PropagateFromSourceToTargetKeypoint(
    int                 edgeId,
    float*              pKeypointCoords,
    int                 sourceKeypointId,
    int                 targetKeypointId,
    float*              pScores,
    float*              pOffsets,
    float*              pDisplacements,
    PosePPConfig*       pPosePPConfig,
    PartWithFloatCoord* pCurPart)
{
    int                    height = pPosePPConfig->featureHeight;
    int                     width = pPosePPConfig->featureWidth;
    int              outputStride = pPosePPConfig->outputStride;
    int              numKeypoints = pPosePPConfig->numKeypoint;
    int                  numEdges = numKeypoints - 1;
    float sourceKeypointCoords[2] = { pKeypointCoords[sourceKeypointId * 2], pKeypointCoords[sourceKeypointId * 2 + 1] };
    int  sourceKeypointIndices[2] = { 0 };
    float       displacedPoint[2] = { 0.0f };
    int  displacedPointIndices[2] = { 0 };

    sourceKeypointIndices[0] = max(0, min(round(sourceKeypointCoords[0] / outputStride), height - 1));
    sourceKeypointIndices[1] = max(0, min(round(sourceKeypointCoords[1] / outputStride), width - 1));

    int displacedPointIdx = sourceKeypointIndices[0] * width * numEdges * 2 +
        sourceKeypointIndices[1] * numEdges * 2 + edgeId * 2;

    displacedPoint[0]        = sourceKeypointCoords[0] + pDisplacements[displacedPointIdx];
    displacedPoint[1]        = sourceKeypointCoords[1] + pDisplacements[displacedPointIdx + 1];
    displacedPointIndices[0] = max(0, min(round(displacedPoint[0] / outputStride), height - 1));
    displacedPointIndices[1] = max(0, min(round(displacedPoint[1] / outputStride), width - 1));

    int offsetsIdx = displacedPointIndices[0] * width * numKeypoints * 2 +
        displacedPointIndices[1] * numKeypoints * 2 + targetKeypointId * 2;

    pCurPart->partScore = pScores[displacedPointIndices[0] * width * numKeypoints +
        displacedPointIndices[1] * numKeypoints + targetKeypointId];
    pCurPart->coord[0]  = float(displacedPointIndices[0]) * float(outputStride) + pOffsets[offsetsIdx];
    pCurPart->coord[1]  = float(displacedPointIndices[1]) * float(outputStride) + pOffsets[offsetsIdx + 1];
}

void TFLPoseNet::DecodePose(
    float              rootScore,
    int                rootId,
    float              rootImageCoords[2],
    float*             pRawHeatmaps,
    float*             pOffsets,
    float*             pDisplacementsBwd,
    float*             pDisplacementsFwd,
    PosePPConfig*      pPosePPConfig,
    PoseResult*        pPoseResult,
    ParentChildTurple* pParentChildTurples)
{
    int                      numParts = pPosePPConfig->numKeypoint;
    int                      numEdges = numParts - 1;
    PartWithFloatCoord tmpPart = {};

    for (int i = 0; i < pPosePPConfig->numKeypoint; i++)
    {
        pPoseResult->keypointScore[i]         = 0.0f;
        pPoseResult->keypointCoord[i * 2]     = 0.0f;
        pPoseResult->keypointCoord[i * 2 + 1] = 0.0f;
    }
    pPoseResult->keypointScore[rootId]         = rootScore;
    pPoseResult->keypointCoord[rootId * 2]     = rootImageCoords[0];
    pPoseResult->keypointCoord[rootId * 2 + 1] = rootImageCoords[1];

    // Backward search
    for (int edge = numEdges - 1; edge >= 0; edge--)
    {
        int targetKeypointId = pParentChildTurples[edge].parent;
        int sourceKeypointId = pParentChildTurples[edge].child;

        if ((pPoseResult->keypointScore[sourceKeypointId] > 0.0f) &&
            (pPoseResult->keypointScore[targetKeypointId] == 0.0f))
        {
            PropagateFromSourceToTargetKeypoint(edge,
                                                pPoseResult->keypointCoord,
                                                sourceKeypointId,
                                                targetKeypointId,
                                                pRawHeatmaps,
                                                pOffsets,
                                                pDisplacementsBwd,
                                                pPosePPConfig,
                                                &tmpPart);
            pPoseResult->keypointScore[targetKeypointId]         = tmpPart.partScore;
            pPoseResult->keypointCoord[targetKeypointId * 2]     = tmpPart.coord[0];
            pPoseResult->keypointCoord[targetKeypointId * 2 + 1] = tmpPart.coord[1];
        }
    }

    // Forward search
    for (int edge = 0; edge < numEdges; edge++)
    {
        int sourceKeypointId = pParentChildTurples[edge].parent;
        int targetKeypointId = pParentChildTurples[edge].child;

        if ((pPoseResult->keypointScore[sourceKeypointId] > 0.0f) &&
            (pPoseResult->keypointScore[targetKeypointId] == 0.0f))
        {
            PropagateFromSourceToTargetKeypoint(edge,
                                                pPoseResult->keypointCoord,
                                                sourceKeypointId,
                                                targetKeypointId,
                                                pRawHeatmaps,
                                                pOffsets,
                                                pDisplacementsFwd,
                                                pPosePPConfig,
                                                &tmpPart);
            pPoseResult->keypointScore[targetKeypointId]         = tmpPart.partScore;
            pPoseResult->keypointCoord[targetKeypointId * 2]     = tmpPart.coord[0];
            pPoseResult->keypointCoord[targetKeypointId * 2 + 1] = tmpPart.coord[1];
        }
    }
}

float TFLPoseNet::CalculatePoseInstanceScore(
    PoseResult*   pPoseResults,
    int           poseCount,
    int           squaredNmsRadius,
    PoseResult*   pCurPoseResult,
    PosePPConfig* pPosePPConfig)
{
    float notOverlappedScores;
    float sum = 0.0f;

    if (0 != poseCount)
    {
        float distBetweenPoses[TotalKeypointNum] = { 0.0f };
        int   flagNMSKeypoints[TotalKeypointNum] = { 0 };
        int                              tmpIdxX = 0;
        int                              tmpIdxY = 0;
        int                               tmpIdx = 0;

        int* flagNMS = static_cast<int*>(malloc(poseCount * TotalKeypointNum * sizeof(int)));
        if (NULL == flagNMS)
        {
            MLE_LOGE(" Error: Couldn't allocate flagNMS buffer for PoseNet\n");
            exit(0);
        }

        memset(flagNMS, 0, poseCount * pPosePPConfig->numKeypoint * sizeof(int));
        memset(flagNMSKeypoints, 1, pPosePPConfig->numKeypoint * sizeof(int));

        // Calculate non-overlapped scores and apply non-maximum surpression (NMS) between poses
        for (int i = 0; i < poseCount; i++)
        {
            for (int j = 0; j < pPosePPConfig->numKeypoint; j++)
            {
                tmpIdx          = i * pPosePPConfig->numKeypoint + j;
                flagNMS[tmpIdx] = 0;
                tmpIdxY         = j * 2;
                tmpIdxX         = j * 2 + 1;

                distBetweenPoses[j] =
                    pow(pPoseResults[i].keypointCoord[tmpIdxY] - pCurPoseResult->keypointCoord[tmpIdxY], 2) +
                    pow(pPoseResults[i].keypointCoord[tmpIdxX] - pCurPoseResult->keypointCoord[tmpIdxX], 2);
                if (distBetweenPoses[j] > float(squaredNmsRadius))
                {
                    flagNMS[tmpIdx] = 1;
                }
            }
        }

        for (int j = 0; j < pPosePPConfig->numKeypoint; j++)
        {
            for (int i = 0; i < poseCount; i++)
            {
                tmpIdx = i * pPosePPConfig->numKeypoint + j;
                if (0 == flagNMS[tmpIdx])
                {
                    flagNMSKeypoints[j] = 0;
                }
            }

            if (flagNMSKeypoints[j])
            {
                sum += pCurPoseResult->keypointScore[j];
            }
        }
        notOverlappedScores = sum / pPosePPConfig->numKeypoint;

        free(flagNMS);

    }
    else
    {
        for (int i = 0; i < pPosePPConfig->numKeypoint; i++)
        {
            sum += pCurPoseResult->keypointScore[i];
        }
        notOverlappedScores = sum / pPosePPConfig->numKeypoint;
    }

    return notOverlappedScores;
}

}; // namespace mle