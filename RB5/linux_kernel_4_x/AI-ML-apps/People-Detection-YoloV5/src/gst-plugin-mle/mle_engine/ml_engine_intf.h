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

#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <time.h>
#include <gst/video/c2d-video-converter.h>
#include <gst/video/gstimagepool.h>
#include <inc/fastcv.h>
#include <ml-meta/ml_meta.h>

namespace mle {

#define GST_TYPE_MLE_INPUT_FORMAT (mle::gst_mle_input_format_get_type())
#define GST_TYPE_MLE_PREPROCESSING_MODE (mle::gst_mle_preprocessing_mode_get_type())
#define GST_TYPE_MLE_SNPE_RUNTIME_TYPE (mle::gst_mle_snpe_runtime_type_get_type())
#define GST_TYPE_MLE_PREPROCESSING_ACCEL (mle::gst_mle_preprocessing_accel_get_type())
#define GST_TYPE_MLE_TFLITE_DELEGATE_TYPE (mle::gst_mle_tflite_delegate_type_get_type())

GType gst_mle_input_format_get_type (void);
GType gst_mle_preprocessing_mode_get_type (void);
GType gst_mle_snpe_runtime_type_get_type (void);
GType gst_mle_preprocessing_accel_get_type (void);
GType gst_mle_tflite_delegate_type_get_type (void);

/*
Input Format
  kRgb - RGB
  kBgr - BGR
  kRgbFloat RGB Float
  kBgrFloat - BGR Float
 */
enum {
  kRgb,
  kBgr,
  kRgbFloat,
  kBgrFloat
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
enum {
  kKeepARCrop,
  kKeepARPad,
  kDirectDownscale,
  kMax
};

/*
Runtime supported by SNPE
  kSnpeCpu - CPU runtime
  kSnpeDsp - DSP runtime
  kSnpeGpu - GPU runtime
  kSnpeAip - AIP runtime
 */
enum {
  kSnpeCpu,
  kSnpeDsp,
  kSnpeGpu,
  kSnpeAip
};

/*
Preprocess Options
  kPrerocessCpu - FastCV CPU performance mode
  kPrerocessDsp - FastCV performance mode
  kPrerocessGpu - C2D preprocess
 */
enum {
  kPreprocessCpu,
  kPreprocessDsp,
  kPreprocessGpu
};

/*
Delegate supported by TFLite
  kTfliteNnapi - NNAPI delegate no predefine accelerator
  kTfliteNnapiNpu - NNAPI delegate NPU accelerator
  kTfliteHexagonNn - Hexagon NN delegate
  kTfliteGpu - GPU delegate
  kTfliteXnnpack - XNN Pack delegate
  kTfliteCpu - CPU runtime
 */
enum {
  kTfliteNnapi,
  kTfliteNnapiNpu,
  kTfliteHexagonNn,
  kTfliteGpu,
  kTfliteXnnpack,
  kTfliteCpu
};

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

enum class NetworkIO {
  kUserBuffer = 0,
  kITensor
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

struct MLConfig {

  //applicable to SNPE
  NetworkIO io_type;

  //Input image format for the desired network
  uint32_t input_format;

  //Aspect ratio maintenance
  uint32_t preprocess_mode;
  uint32_t preprocess_accel;

  // normalization
  float blue_mean;
  float blue_sigma;
  float green_mean;
  float green_sigma;
  float red_mean;
  float red_sigma;
  bool use_norm;
  // end normalization

  // coordinate for people intrusion detection
  uint32_t x_axis;
  uint32_t y_axis;
  uint32_t width;
  uint32_t height;

  float conf_threshold;
  float nms_threshold;
  uint32_t max_detection_result;

  std::string model_file;
  std::string labels_file;

  //snpe specific
  uint32_t runtime;

  //tflite specific
  uint32_t number_of_threads;
  uint32_t delegate;

  //snpe layers
  std::vector<std::string> output_layers;
};

class MLEngine {
 public:
  MLEngine(MLConfig &config);
  virtual ~MLEngine(){};
  int32_t Init(const MLEInputParams* source_info);
  virtual void Deinit();
  int32_t PreProcess(GstVideoFrame *frame);
  int32_t Process(GstVideoFrame *frame);
 private:
  virtual int32_t LoadModel(std::string& model_path) = 0;
  virtual int32_t InitFramework() = 0;
  virtual int32_t ExecuteModel() = 0;
  virtual void* GetInputBuffer() = 0;
  virtual int32_t PostProcess(GstBuffer* buffer) = 0;
  int32_t ReadLabelsFile(const std::string& file_name,
                        std::vector<std::string>& result,
                        size_t& found_label_count);
  void PreProcessAccelerator();
  static bool fastcv_mode_is_set_;
  static std::mutex fastcv_process_lock_;

 protected:

  virtual int32_t AllocateInternalBuffers();
  virtual void FreeInternalBuffers();

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

  void MeanSubtract(uint8_t* input_buf,
                    const uint32_t width,
                    const uint32_t height,
                    const uint32_t pad_width,
                    const uint32_t pad_height,
                    float* processed_buf);

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
  uint32_t batch_size_;
  std::vector<std::string> labels_;
  size_t label_count_;
  GstC2dVideoConverter *c2dconvert_;
  GstBufferPool *outpool_;
  static bool use_c2d_preprocess_;
  GstVideoFrame *c2d_buf_outframe_;
  GstBuffer *gst_c2d_buf_;
};

}; // namespace mle
