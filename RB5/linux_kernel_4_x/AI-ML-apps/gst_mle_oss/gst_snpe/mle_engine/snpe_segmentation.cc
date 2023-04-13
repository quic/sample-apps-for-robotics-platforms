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
#include <cmath>
#include "snpe_segmentation.h"
#include "common_utils.h"

namespace mle {

static const uint32_t       kOutBytesPerPixel = 4;

SNPESegmentation::SNPESegmentation(MLConfig &config) : SNPEBase(config) {
  need_labels_ = false;
}
SNPESegmentation::~SNPESegmentation() {}

int32_t SNPESegmentation::PostProcess(GstBuffer* buffer) {
  std::vector<float> segm_buf;

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

          if (0 == std::strcmp(name, snpe_params_.result_layers[0].c_str())) {
            segm_buf = snpe_params_.out_heap_map.at(name);
          }
        } else if (config_.io_type == NetworkIO::kITensor) {
          MLE_LOGE("ITensor currently not supported");
        }
      });

  if (buffer) {
    GstMLSegmentationMeta *img_meta =
        gst_buffer_add_segmentation_meta (buffer);
    if (!img_meta) {
      MLE_LOGE ("Failed to add overlay image meta");
      return MLE_FAIL;
    }

    uint32_t image_size = scale_width_ * scale_height_ * kOutBytesPerPixel;

    if (img_meta->img_buffer == nullptr) {
      img_meta->img_buffer = (gpointer) calloc (1, image_size);
      if (!img_meta->img_buffer) {
        MLE_LOGE(" Failed to allocate image buffer");
        return MLE_FAIL;
      }
    }

    img_meta->img_width  = scale_width_;
    img_meta->img_height = scale_height_;
    img_meta->img_size   = image_size;
    img_meta->img_format = GST_VIDEO_FORMAT_RGBA;
    img_meta->img_stride = scale_width_ * kOutBytesPerPixel;

    if (segm_buf.size()) {
      for (uint32_t y = 0; y < scale_height_; y++) {
        for (uint32_t x = 0; x < scale_width_; x++) {
          uint32_t label =
           static_cast<uint32_t>(segm_buf[y * engine_input_params_.width + x]);
          ((uint32_t*)img_meta->img_buffer)[y * scale_width_ + x] =
              static_cast<uint32_t>(color_table[label].red)  |
              static_cast<uint32_t>(color_table[label].green) << 8 |
              static_cast<uint32_t>(color_table[label].blue) << 16 |
              static_cast<uint32_t>(color_table[label].alpha) << 24;
        }
      }
    }
  }

  return MLE_OK;
}

}; // namespace mle
