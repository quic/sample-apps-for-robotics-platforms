/*
* Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
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

#include <vector>
#include <cmath>
#include "snpe_detection.h"
#include "common_utils.h"

namespace mle {

SNPEDetection::SNPEDetection(MLConfig &config) : SNPEBase(config) {}
SNPEDetection::~SNPEDetection() {}

int32_t SNPEDetection::PostProcess(GstBuffer* buffer) {
  MLE_LOGI("%s: Enter", __func__);

  std::vector<float> score_buf;
  std::vector<float> box_buf;
  std::vector<float> class_buf;

  const zdl::DlSystem::StringList &output_buf_names =
      snpe_params_.output_ub_map.getUserBufferNames();
  const zdl::DlSystem::StringList &output_tensor_names =
      snpe_params_.output_tensor_map.getTensorNames();
  const zdl::DlSystem::StringList *output_names = &output_buf_names;
  if (config_.io_type == NetworkIO::kITensor) {
    output_names = &output_tensor_names;
  }
  std::for_each(
      output_names->begin(),
      output_names->end(),
      [&](const char* name)
      {
        if (config_.io_type == NetworkIO::kUserBuffer) {
          if (0 == std::strcmp(name, snpe_params_.result_layers[2].c_str())) {
            score_buf = snpe_params_.out_heap_map.at(name);
          } else if (0 == std::strcmp(
                          name, snpe_params_.result_layers[0].c_str())) {
            box_buf = snpe_params_.out_heap_map.at(name);
          } else if (0 == std::strcmp(
                          name, snpe_params_.result_layers[3].c_str())) {
            class_buf = snpe_params_.out_heap_map.at(name);
          }
        } else if (config_.io_type == NetworkIO::kITensor) {
          if (0 == std::strcmp(name, snpe_params_.result_layers[0].c_str())) {
            auto t = snpe_params_.output_tensor_map.getTensor(name);
            for (auto it = t->begin(); it != t->end(); it++) {
              score_buf.push_back(*it);
            }
          } else if (0 == std::strcmp(
                          name, snpe_params_.result_layers[1].c_str())) {
            auto t = snpe_params_.output_tensor_map.getTensor(name);
            for (auto it = t->begin(); it != t->end(); it++) {
              box_buf.push_back(*it);
            }
          } else if (0 == std::strcmp(
                          name, snpe_params_.result_layers[2].c_str())) {
            auto t = snpe_params_.output_tensor_map.getTensor(name);
            for (auto it = t->begin(); it != t->end(); it++) {
              class_buf.push_back(*it);
            }
          }
        }
      });

  uint32_t width = source_params_.width;
  uint32_t height = source_params_.height;

  float scale_ratio_x = (float)engine_input_params_.width / scale_width_;
  float scale_ratio_y = (float)engine_input_params_.height / scale_height_;

  if (config_.preprocess_mode == kKeepARCrop) {
    width = po_.width;
    height = po_.height;
  }

  if (score_buf.size() && box_buf.size() && class_buf.size()) {
    uint32_t num_obj = 0;
    for (size_t i = 0; i < score_buf.size(); i++) {
      if (score_buf[i] < config_.conf_threshold) {
        continue;
      }
      GstMLDetectionMeta *meta = gst_buffer_add_detection_meta(buffer);
      if (!meta) {
        MLE_LOGE("Failed to create metadata");
        return MLE_NULLPTR;
      }

      GstMLClassificationResult *box_info = (GstMLClassificationResult*)malloc(
          sizeof(GstMLClassificationResult));

      uint32_t label_size = labels_.at(
          static_cast<uint32_t>(class_buf[i] + 0.5)).size() + 1;
      box_info->name = (gchar *)malloc(label_size);
      snprintf(box_info->name, label_size, "%s",
               labels_.at(static_cast<uint32_t>(class_buf[i] + 0.5)).c_str());
      box_info->confidence = score_buf[i];
      meta->box_info = g_slist_append (meta->box_info, box_info);

      meta->bounding_box.x =
        std::lround(box_buf[i * 4 + 1] * width * scale_ratio_x) + po_.x_offset;
      meta->bounding_box.y =
        std::lround(box_buf[i * 4] * height * scale_ratio_y) + po_.y_offset;
      meta->bounding_box.width =
        (std::lround(box_buf[i * 4 + 3] * width * scale_ratio_x) +
                     po_.x_offset) - meta->bounding_box.x;
      meta->bounding_box.height =
        (std::lround(box_buf[i * 4 + 2] * height * scale_ratio_y) +
                     po_.y_offset) - meta->bounding_box.y;

    num_obj++;

    MLE_LOGD("object info: name: %s , score %f, box x %d y %d w %d h %d",
                box_info->name, box_info->confidence,
                meta->bounding_box.x,
                meta->bounding_box.y,
                meta->bounding_box.width,
                meta->bounding_box.height);
    }
    MLE_LOGI("Inference engine detected %d objects, highest score: %f",
                  num_obj, score_buf[0]);
  }

  MLE_LOGI("%s: Exit", __func__);
  return MLE_OK;
}

}; // namespace mle
