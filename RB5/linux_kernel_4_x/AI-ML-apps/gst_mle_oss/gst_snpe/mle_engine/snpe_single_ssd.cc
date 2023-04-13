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

#include <vector>
#include "snpe_single_ssd.h"

namespace mle {

static const int kMaxNumObjects = 5;

SNPESingleSSD::SNPESingleSSD(MLConfig &config) : SNPEBase(config),
                                                 output_params_(nullptr) {}
SNPESingleSSD::~SNPESingleSSD() {}

int32_t SNPESingleSSD::PostProcess(GstBuffer* buffer) {
  MLE_LOGI("%s: Enter", __func__);

  std::vector<float> result_buf;
  const zdl::DlSystem::StringList &output_buf_names =
      snpe_params_.output_ub_map.getUserBufferNames();
  const zdl::DlSystem::StringList *output_names = &output_buf_names;

  std::for_each(
      output_names->begin(),
      output_names->end(),
      [&](const char* name)
      {
    // Currently, singleSSD supports only UserBuffers
        if (config_.io_type == NetworkIO::kUserBuffer) {
          if (0 == std::strcmp(name, snpe_params_.result_layers[0].c_str())) {
            result_buf = snpe_params_.out_heap_map.at(name);
          }
        }
      });

  if (result_buf.size()) {
    output_params_ = reinterpret_cast<OutputParams*>(result_buf.data());

    uint32_t width = source_params_.width;
    uint32_t height = source_params_.height;

    if (config_.preprocess_mode == PreprocessingMode::kKeepARCrop) {
      width = po_.width;
      height = po_.height;
    }

    for (size_t i = 0; i < kMaxNumObjects; i++) {
      if (output_params_[i].score < config_.conf_threshold) {
        continue;
      }
      if ((output_params_[i].boxes.x_min < 0) ||
          (output_params_[i].boxes.y_min < 0) ||
          (output_params_[i].boxes.x_max < 0) ||
          (output_params_[i].boxes.y_max < 0)) {
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
          static_cast<uint32_t>(output_params_[i].label + 0.5)).size() + 1;
      box_info->name = (gchar *)malloc(label_size);
      snprintf(
          box_info->name,
          label_size,
          "%s",
          labels_.at(
              static_cast<uint32_t>(output_params_[i].label + 0.5)).c_str());
      box_info->confidence = output_params_[i].score;
      meta->box_info = g_slist_append (meta->box_info, box_info);

      meta->bounding_box.x =
          std::lround(output_params_[i].boxes.x_min * width) + po_.x_offset;
      meta->bounding_box.y =
          std::lround(output_params_[i].boxes.y_min * height) + po_.y_offset;
      meta->bounding_box.width =
          (std::lround(output_params_[i].boxes.x_max * width) + po_.x_offset) -
              meta->bounding_box.x;
      meta->bounding_box.height =
          (std::lround(output_params_[i].boxes.y_max * width) + po_.x_offset) -
              meta->bounding_box.y;
    }
  }

  MLE_LOGI("%s: Exit", __func__);
  return MLE_OK;
}

size_t SNPESingleSSD::CalculateSizeFromDims(const size_t rank,
                                       const zdl::DlSystem::Dimension* dims,
                                       const size_t& element_size) {
  if (0 == rank) {
    return 0;
  }
  size_t size = element_size;
  for (size_t i = 0; i < rank; i++) {
    if (0 == dims[i]) {
      size *= kMaxNumObjects;
      continue;
    }
    size *= dims[i];
  }
  return size;
}

std::vector<size_t> SNPESingleSSD::GetStrides(zdl::DlSystem::TensorShape dims,
                                              const size_t& element_size) {
  std::vector<size_t> strides(dims.rank());
  strides[strides.size() - 1] = element_size;
  size_t stride = strides[strides.size() - 1];

  for (size_t i = dims.rank() - 1; i > 0; i--) {
    if (0 == dims[i]) {
      stride *= kMaxNumObjects;
    } else {
      stride *= dims[i];
    }
    strides[i - 1] = stride;
  }

  return strides;
}

}; // namespace mle
