/*
Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the
disclaimer below) provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <vector>
#include <cmath>
#include "snpe_yolodetection.h"
#include "common_utils.h"

//#include <math.h>

//#include <algorithm>
//#include <assert.h>
//#include <iostream>
//#include <queue>

namespace Algorithm {

static float fastPow(float p) {
  float offset = (p < 0) ? 1.0f : 0.0f;
  float clipp = (p < -126) ? -126.0f : p;
  int32_t w = (int32_t)clipp;
  float z = clipp - w + offset;
  union {
    uint32_t i;
    float f;
  } v = {(uint32_t)((1 << 23) *
                    (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) -
                     1.49012907f * z))};

  return v.f;
}

inline float fastExp(float p) { return fastPow(1.442695040f * p); }

inline float Sigmoid(float x)
{
    return (1 / (1 + exp(-x)));
}

inline float fastSigmoid(float x)
{
    return (1 / (1 + fastExp(-x)));
}

} // namespace Algorithm

namespace mle {

const float strides[3]= {8,16,32};
const float anchors[3][6] = {{10, 13, 16, 30, 33, 23}, {30, 61, 62, 45, 59, 119}, {116, 90, 156, 198, 373, 326}};

std::vector<float> boundingBoxDecode(const float *pbox, float x_o, float y_o, float anchor_w, float anchor_h, float stride) {
  float x, y, xw, yh, x1, x2, y1, y2;
  x = (Algorithm::Sigmoid(pbox[0]) + x_o) * stride;
  y = (Algorithm::Sigmoid(pbox[1]) + y_o) * stride;
  xw = exp(pbox[2]) * anchor_w;
  yh = exp(pbox[3]) * anchor_h;
  x1 = x - xw / 2; if(x1<0) x1=0;
  y1 = y - yh / 2; if(y1<0) y1=0;
  x2 = x + xw / 2;
  y2 = y + yh / 2;
  return {x1, y1, x2, y2};
}

std::vector<float> fastBoundingBoxDecodeV5(const float *pbox, float x_o, float y_o, float anchor_w, float anchor_h, float stride) {
  float x, y, xw, yh, x1, x2, y1, y2;
  x = ( Algorithm::fastSigmoid(pbox[0]) * 2 - 0.5 + x_o) * stride;
  y = ( Algorithm::fastSigmoid(pbox[1]) * 2 - 0.5 + y_o) * stride;
  xw = Algorithm::fastSigmoid(pbox[2]);
  xw = xw * xw * anchor_w * 4;
  yh = Algorithm::fastSigmoid(pbox[3]);
  yh = yh * yh * anchor_h * 4;
  x1 = x - xw / 2; if(x1<0) x1=0;
  y1 = y - yh / 2; if(y1<0) y1=0;
  x2 = x + xw / 2;
  y2 = y + yh / 2;
  return {x1, y1, x2, y2};
}

static float computeIoU(const float *box1, const float *box2) {
  float box1_xmin = box1[0], box2_xmin = box2[0];
  float box1_ymin = box1[1], box2_ymin = box2[1];
  float box1_xmax = box1[2], box2_xmax = box2[2];
  float box1_ymax = box1[3], box2_ymax = box2[3];
  float ixmin = std::max(box1_xmin, box2_xmin);
  float iymin = std::max(box1_ymin, box2_ymin);
  float ixmax = std::min(box1_xmax, box2_xmax);
  float iymax = std::min(box1_ymax, box2_ymax);

  // if(!((box1_xmin <= box1_xmax) && (box1_ymin <= box1_ymax) &&
         // (box2_xmin <= box2_xmax) && (box2_ymin <= box2_ymax))) {
	// return 0;
  // }

  float iou = 0.0f;
  if ((ixmin < ixmax) && (iymin < iymax)) {
    float intersection_area = (ixmax - ixmin) * (iymax - iymin);
    // union = area1 + area2 - intersection
    float union_area = (box1_xmax - box1_xmin) * (box1_ymax - box1_ymin) +
                       (box2_xmax - box2_xmin) * (box2_ymax - box2_ymin) -
                       intersection_area;
    iou = (union_area > 0.0f) ? intersection_area / union_area : 0.0f;
  }
  return iou;
}

void SNPEYoloDetection::doNMS(std::vector<std::vector<float>> &boxes, float thres) {
  // 0,1,2,3 - vertex, 4 - confidence
  using box = std::vector<float>;
  //        std::cout << boxes.size() << std::endl;
  std::sort(boxes.begin(), boxes.end(),
            [](const box &a, const box &b) { return a[4] > b[4]; });

  for (auto it = boxes.begin(); it != boxes.end(); it++) {
    box &cand1 = *it;

    for (auto jt = it + 1; jt != boxes.end();) {
      box &cand2 = *jt;
      if(cand1[5] != cand2[5]){
        jt++;
        continue;   // nms with same class only
      }
      if (computeIoU(&cand1[0], &cand2[0]) >= thres)
        jt = boxes.erase(jt); // Possible candidate for optimization
      else
        jt++;
    }
  }
  // std::cout << "Final number of discrete bounding boxes = "<< boxes.size() <<
  // std::endl;
}

SNPEYoloDetection::SNPEYoloDetection(MLConfig &config) : SNPEBase(config) {}

SNPEYoloDetection::~SNPEYoloDetection() {}

int32_t SNPEYoloDetection::PostProcess(GstBuffer* buffer) {
  MLE_LOGI("%s: Enter.", __func__);

  uint32_t detection_size = (NUM_COORDINATES + 1 + label_count_ ); //4 + 1 + 80
  std::vector<std::vector<float>> res_vec;
  for (uint32_t i = 0; i < snpe_params_.result_layers.size(); i++) {
    const char* name =  snpe_params_.result_layers[i].c_str();
    std::vector<float> _raw_buf;
    if (config_.io_type == NetworkIO::kUserBuffer) {
      _raw_buf= snpe_params_.out_heap_map.at(name);
      MLE_LOGD("%s: out_heap_map: %s. size=%d", __func__,name,(uint32_t)_raw_buf.size());
    } else if (config_.io_type == NetworkIO::kITensor) {
      auto t = snpe_params_.output_tensor_map.getTensor(name);
      for (auto it = t->begin(); it != t->end(); it++) {
       _raw_buf.push_back(*it);
      }
      MLE_LOGD("%s: output_tensor_map: %s. size=%d", __func__,name,(uint32_t)_raw_buf.size());
    }

    uint32_t outbuf_size=_raw_buf.size();
    for (uint32_t n = 0; n < DEFAULT_ANCHOR_BOXES; n++)
    {
      //uint32_t grid_size= engine_input_params_.height/strides[n];
      if (batch_size_*(engine_input_params_.height/strides[n])*(engine_input_params_.width/strides[n])*DEFAULT_ANCHOR_BOXES*detection_size == outbuf_size){
        anchorBoxProcess(n,_raw_buf.data(),res_vec);
      }
    }
  }
  doNMS(res_vec, config_.nms_threshold);
  ShowDetectionOverlay(buffer, res_vec);
  MLE_LOGI("%s: Exit", __func__);
  return MLE_OK;
}

int32_t SNPEYoloDetection::anchorBoxProcess(uint32_t anchorBoxIdx,float *pdata, std::vector<std::vector<float>> &res_vec) {
  float stride = strides[anchorBoxIdx];
  uint32_t grid_height = engine_input_params_.height/stride;
  uint32_t grid_width =  engine_input_params_.width/stride;
  uint32_t detection_size = (NUM_COORDINATES + 1 + label_count_ ); //4 + 1 + 80
  uint32_t cnt = 0;
  for (uint32_t batchIdx = 0; batchIdx < batch_size_; batchIdx++) {
    for (uint32_t h = 0; h < grid_height; h++) {
      for (uint32_t w = 0; w < grid_width; w++) {
        for (uint32_t winIdx = 0; winIdx < 3 ; winIdx++) {
          float *pbox = pdata + cnt*detection_size;
          float *pclass = pbox + NUM_COORDINATES + 1;
          cnt++;
          float objectConfidenceScore =Algorithm::fastSigmoid(pbox[4]);
          if (objectConfidenceScore <= config_.conf_threshold) {
            continue;
          }
          uint32_t max_class_index =0;
          if(label_count_ > 1){
            max_class_index = std::distance(pclass, std::max_element(pclass, pclass + label_count_));
          }
          float max_objprob =Algorithm::fastSigmoid(pclass[max_class_index]);
          if (max_objprob * objectConfidenceScore <= config_.conf_threshold) {
            continue;
          }
          float anchor_w= anchors[anchorBoxIdx][winIdx * 2];
          float anchor_h = anchors[anchorBoxIdx][(winIdx * 2) + 1];
          std::vector<float> box = fastBoundingBoxDecodeV5(pbox, (float)w, (float)h, anchor_w, anchor_h,stride);
          res_vec.emplace_back(std::initializer_list<float>{
              box[0], box[1], box[2],box[3], max_objprob * objectConfidenceScore,(float)max_class_index});
        }
      }
    }
  }
  MLE_LOGI("anchorBoxIdx:%d grid:%dx%d: total_boxes:%d  cnt:%d ",anchorBoxIdx, grid_width,grid_height,(int)res_vec.size(),cnt);
  return MLE_OK;
}

int32_t SNPEYoloDetection::ShowDetectionOverlay(GstBuffer* buffer, std::vector<std::vector<float>> &res_vec)
{
  uint32_t width = source_params_.width;
  uint32_t height = source_params_.height;

  if (config_.preprocess_mode == kKeepARCrop) {
    width = po_.width;
    height = po_.height;
  }
  uint32_t total_boxes =res_vec.size();
  MLE_LOGI("width:%d: height:%d x_offset:%d y_offset:%d  total_boxes:%d ",width,height,po_.x_offset,po_.y_offset,total_boxes);

  if(total_boxes > config_.max_detection_result) total_boxes = config_.max_detection_result;
  for (uint32_t i = 0; i < total_boxes; i++)
  {
    float confidence= res_vec[i][4];
    float class_idx= res_vec[i][5];
    if (class_idx>=label_count_) continue;
    // Only interested in people detection, Discard other classes
    if(class_idx!=0.0) continue;
    std::string class_name= labels_.at(static_cast<uint32_t>(class_idx));
    uint32_t label_size = class_name.size() + 1;

    uint32_t bx= std::lround(res_vec[i][0] * width /scale_width_) +  po_.x_offset;
    uint32_t by= std::lround(res_vec[i][1] * height /scale_height_) + po_.y_offset;
    uint32_t bw= std::lround(res_vec[i][2] * width /scale_width_) +  po_.x_offset - bx;
    uint32_t bh= std::lround(res_vec[i][3] * height /scale_height_) + po_.y_offset - by;

	if ((config_.x_axis != 0) || (config_.y_axis != 0) || (config_.width != 0) ||
			(config_.height != 0)) {
		if (!(((bx > config_.x_axis) && (bx < config_.x_axis + config_.width)) ||
					((bx + bw > config_.x_axis) && (bx + bw < config_.x_axis + config_.width)))) {
			continue;
		}
		
		if (!(((by > config_.y_axis) && (by < config_.y_axis + config_.height)) ||
					((by + bh > config_.y_axis) && (by + bh < config_.y_axis + config_.height)))) {
			continue;
		}
	}		


    MLE_LOGI("object info: name: %s , score %3f, box x %d y %d w %d h %d",
              class_name.c_str(), confidence, bx, by, bw, bh);

    GstMLDetectionMeta *meta = gst_buffer_add_detection_meta(buffer);
    if (!meta) {
      MLE_LOGE("Failed to create metadata");
      return MLE_NULLPTR;
    }

    GstMLClassificationResult *box_info = (GstMLClassificationResult*)malloc(
        sizeof(GstMLClassificationResult));

    box_info->name = (gchar *)malloc(label_size);
    snprintf(box_info->name, label_size, "%s", class_name.c_str());
    box_info->confidence = confidence;
    meta->box_info = g_slist_append (meta->box_info, box_info);
    meta->bounding_box.x = bx;
    meta->bounding_box.y = by;
    meta->bounding_box.width = bw;
    meta->bounding_box.height = bh;

  }
  return MLE_OK;

}

}; // namespace mle
