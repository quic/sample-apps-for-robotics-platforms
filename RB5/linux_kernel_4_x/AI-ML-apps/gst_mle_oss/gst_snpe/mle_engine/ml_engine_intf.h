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

#pragma once

#include <vector>
#include <string>
#include <time.h>
#include <ml-meta/ml_meta.h>
#include "common_utils.h"

namespace mle {

enum MLEImageFormat {
  mle_format_invalid = 0,
  mle_format_nv12,
  mle_format_nv21,
  mle_format_RGB24,
};

enum MLEErrors {
  MLE_OK = 0,
  MLE_FAIL,
  MLE_NULLPTR,
  MLE_IMG_FORMAT_NOT_SUPPORTED
};

enum class RuntimeType {
  CPU = 0,
  DSP,
  GPU,
  AIP
};

enum class InputFormat {
  kRgb = 0,
  kBgr,
  kRgbFloat,
  kBgrFloat
};

enum class NetworkIO {
  kUserBuffer = 0,
  kITensor
};

/*
MLE supports three pre-processing modes
  kKeepARCrop - This mode crops from the original frame to match engine's input
            aspect ratio. Objects outside the crop region will not be detected
  kKeepARPad - This mode keeps original frame aspect ratio. In order to match
             engine's input requirements, padding with mean value is added
  kDirectDownscale - This mode doesn't keep the aspect ratio of the original
                      frame and shrinks it
*/

enum class PreprocessingMode {
  kKeepARCrop = 0,
  kKeepARPad,
  kDirectDownscale,
  kMax
};

struct PreprocessingOffsets {
  PreprocessingOffsets(): x_offset(0),
                          y_offset(0),
                          width(0),
                          height(0) {};
  uint32_t x_offset;
  uint32_t y_offset;
  uint32_t width;
  uint32_t height;
};

struct PreprocessingBuffers {
  uint8_t* scale_buf;
  uint8_t* rgb_buf;
};

struct MLEInputParams {
  uint32_t width;
  uint32_t height;
  MLEImageFormat format;
};

struct SourceFrame {
  uint8_t *frame_data[2];
  uint32_t stride;
};

struct MLConfig {

  //applicable to SNPE
  NetworkIO io_type;

  //Input image format for the desired network
  InputFormat input_format;

  //Aspect ratio maintenance
  PreprocessingMode preprocess_mode;

  // normalization
  float blue_mean;
  float blue_sigma;
  float green_mean;
  float green_sigma;
  float red_mean;
  float red_sigma;
  bool use_norm;
  // end normalization

  float conf_threshold;
  std::string model_file;
  std::string labels_file;

  //runtime
  RuntimeType runtime;

  //tflite specific
  uint32_t number_of_threads;
  std::string delegate;

  //snpe layers
  std::vector<std::string> output_layers;
};

class MLEngine {
 public:
  MLEngine(MLConfig &config);
  virtual ~MLEngine(){};
  int32_t Init(const MLEInputParams* source_info);
  virtual void Deinit();
  int32_t PreProcess(const struct SourceFrame* frame_info);
  int32_t Process(struct SourceFrame* frame_info,
                          GstBuffer* buffer);
 private:
  virtual int32_t LoadModel(std::string& model_path) = 0;
  virtual int32_t InitFramework() = 0;
  virtual int32_t ExecuteModel() = 0;
  virtual void* GetInputBuffer() = 0;
  virtual int32_t PostProcess(GstBuffer* buffer) = 0;
  int32_t ReadLabelsFile(const std::string& file_name,
                        std::vector<std::string>& result,
                        size_t& found_label_count);
  virtual int32_t AllocateInternalBuffers();
  virtual void FreeInternalBuffers();
 protected:

  void DumpFrame(const uint8_t* buffer, const uint32_t& width,
      const uint32_t& height, const uint32_t& size, const std::string& suffix);

  void Pad(
      uint8_t*       input_buf,
      const uint32_t input_width,
      const uint32_t input_height,
      const uint32_t pad_width,
      const uint32_t pad_height,
      uint8_t*       output_buf);

  void PreProcessColorConvertRGB(
      uint8_t*       pSrcLuma,
      uint8_t*       pSrcChroma,
      uint8_t*       pDst,
      const uint32_t width,
      const uint32_t height,
      MLEImageFormat format);

  void PreProcessColorConvertBGR(
      uint8_t*       pSrc,
      uint8_t*       pDst,
      const uint32_t width,
      const uint32_t height);

  int32_t PreProcessScale(
      uint8_t*       pSrcLuma,
      uint8_t*       pSrcChroma,
      uint8_t*       pDst,
      const uint32_t srcWidth,
      const uint32_t srcHeight,
      const uint32_t srcStride,
      const uint32_t scaleWidth,
      const uint32_t scaleHeight,
      MLEImageFormat format);

  void MeanSubtract(uint8_t* input_buf, const uint32_t width,
                    const uint32_t height, float* processed_buf);

  MLConfig config_;
  MLEInputParams source_params_;
  //params for engine's input requirements
  MLEInputParams engine_input_params_;
  PreprocessingBuffers buffers_;
  PreprocessingOffsets po_;
  uint32_t scale_width_;
  uint32_t scale_height_;
  bool do_rescale_;
  bool need_labels_;
  std::vector<std::string> labels_;
  size_t label_count_;

};

class Timer {
  std::string str;
  uint64_t begin;

public:

  Timer (std::string s) : str(s) {
    begin = GetMicroSeconds();
  }

  ~Timer () {
    uint64_t end = GetMicroSeconds();
    MLE_LOGD("%s: %llu us", str.c_str(),
        static_cast<long long unsigned>(end - begin));
  }

  uint64_t GetMicroSeconds()
{
  timespec time;

  clock_gettime(CLOCK_MONOTONIC, &time);

  uint64_t microSeconds = (static_cast<uint32_t>(time.tv_sec) * 1000000ULL) +
                          (static_cast<uint32_t>(time.tv_nsec)) / 1000;

  return microSeconds;
}
};

}; // namespace mle
