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

#include <string>
#include <fstream>
#include <vector>
#include <memory>
#include <sstream>
#include <math.h>
#include "ml_engine_intf.h"
#include "common_utils.h"

namespace mle {

bool MLEngine::fastcv_mode_is_set_ = false;
bool MLEngine::use_c2d_preprocess_ = false;
std::mutex MLEngine::fastcv_process_lock_;

MLEngine::MLEngine(MLConfig &config) : config_(config) {
  std::lock_guard<std::mutex> lock(fastcv_process_lock_);
  if (!fastcv_mode_is_set_) {
    PreProcessAccelerator();
    fastcv_mode_is_set_ = true;
  }

  buffers_.scale_buf = nullptr;
  buffers_.rgb_buf = nullptr;
  gst_c2d_buf_ = nullptr;
  outpool_ = nullptr;
  c2d_buf_outframe_ = nullptr;
}

void MLEngine::PreProcessAccelerator() {
  fcvOperationMode mode = FASTCV_OP_CPU_PERFORMANCE;
  use_c2d_preprocess_ = false;

  switch(config_.preprocess_accel) {
    case kPreprocessGpu:
      MLE_LOGI("%s Use C2D for postprocessing", __func__);
      use_c2d_preprocess_ = true;
      break;
    case kPreprocessDsp:
      MLE_LOGI("%s FastCV operation is PERFORMANCE", __func__);
      mode = FASTCV_OP_PERFORMANCE;
      break;
    case kPreprocessCpu:
    default:
      MLE_LOGI("%s FastCV operation is CPU PERFORMANCE", __func__);
      break;
  }

  fcvSetOperationMode(mode);
}

void MLEngine::DumpFrame(const uint8_t* buffer, const uint32_t& width,
    const uint32_t& height, const uint32_t& size, const std::string& suffix) {

  std::string file_path("/data/misc/camera/ml_engine_");
  size_t written_len = 0;
  file_path += std::to_string(width);
  file_path += "x";
  file_path += std::to_string(height);
  file_path += suffix;
  FILE *file = fopen(file_path.c_str(), "w+");
  if (!file) {
    MLE_LOGE("%s: Unable to open file(%s)", __func__,
        file_path.c_str());
    goto FAIL;
  }
  written_len = fwrite(buffer, sizeof(uint8_t), size, file);
  MLE_LOGD("%s: written_len: %zu", __func__, written_len);
  if (size != written_len) {
    MLE_LOGE("%s: Bad Write error (%d):(%s)", __func__, errno,
             strerror(errno));
    goto FAIL;
  }
  MLE_LOGD("%s: Buffer Size:%zu Stored:%s", __func__, written_len,
    file_path.c_str());

FAIL:
  if (file != nullptr) {
    fclose(file);
  }
}

int32_t MLEngine::AllocateInternalBuffers() {

  if (do_rescale_) {
    GstBufferPool *pool = NULL;
    GstStructure *config = NULL;
    GstAllocator *allocator = NULL;
    GstVideoInfo vinfo;
    uint32_t size = ((scale_width_ * scale_height_ * 4));
    uint32_t size_aligned = (size + 4096-1) & ~(4096-1);

    GstCaps *caps = gst_caps_new_simple ("video/x-raw",
          "format", G_TYPE_STRING, "RGBA",
          "framerate", GST_TYPE_FRACTION, 25, 1,
          "width", G_TYPE_INT, scale_width_,
          "height", G_TYPE_INT, scale_height_,
          NULL);

    pool = gst_image_buffer_pool_new (GST_IMAGE_BUFFER_POOL_TYPE_GBM);
    if (pool == NULL) {
      MLE_LOGE("%s: Failed create buffer image pool", __func__);
      return MLE_FAIL;
    }

    config = gst_buffer_pool_get_config (pool);
    if (config == NULL) {
      MLE_LOGE("%s: Failed set config of the pool", __func__);
      gst_object_unref (pool);
      return MLE_FAIL;
    }

    gst_buffer_pool_config_set_params (config, caps, size_aligned, 1, 1);

    allocator = gst_fd_allocator_new ();
    gst_buffer_pool_config_set_allocator (config, allocator, NULL);

    if (!gst_buffer_pool_set_config (pool, config)) {
      MLE_LOGE("%s: Failed to set pool configuration", __func__);
      g_object_unref (pool);
      g_object_unref (allocator);
      return MLE_FAIL;
    }

    g_object_unref (allocator);
    outpool_ = pool;

    if (!gst_buffer_pool_is_active (pool) &&
        !gst_buffer_pool_set_active (pool, TRUE)) {
      MLE_LOGE("%s: Failed to activate output video buffer pool", __func__);
      g_object_unref (outpool_);
      outpool_ = nullptr;
      return MLE_FAIL;
    }

    if (GST_FLOW_OK != gst_buffer_pool_acquire_buffer (pool,
        &gst_c2d_buf_, NULL)) {
      MLE_LOGE("%s: Failed to create output buffer", __func__);
      g_object_unref (outpool_);
      outpool_ = nullptr;
      return MLE_FAIL;
    }

    c2d_buf_outframe_ = g_slice_new (GstVideoFrame);

    GstVideoFormat format = GST_VIDEO_FORMAT_RGB;
    if (config_.input_format == kBgr ||
        config_.input_format == kBgrFloat) {
      format = GST_VIDEO_FORMAT_BGR;
    }
    gst_video_info_set_format (&vinfo, format, scale_width_, scale_height_);

    if (!gst_video_frame_map (c2d_buf_outframe_, &vinfo, gst_c2d_buf_,
            (GstMapFlags)(GST_MAP_READWRITE |
            GST_VIDEO_FRAME_MAP_FLAG_NO_REF))) {
      MLE_LOGE("Failed to map buffer");
      FreeInternalBuffers();
      return MLE_FAIL;
    }

    buffers_.rgb_buf =
        (uint8_t*) GST_VIDEO_FRAME_PLANE_DATA (c2d_buf_outframe_, 0);
  }

  if (!use_c2d_preprocess_) {
    posix_memalign(reinterpret_cast<void**>(&buffers_.scale_buf),
                                    128,
                                    ((scale_width_ *
                                          scale_height_ * 3 / 2)));
    if (nullptr == buffers_.scale_buf) {
      MLE_LOGE("%s: Scale buf allocation failed", __func__);
      FreeInternalBuffers();
      return MLE_FAIL;
    }
  }
  return MLE_OK;
}

void MLEngine::FreeInternalBuffers() {
  if (nullptr != c2d_buf_outframe_) {
    gst_video_frame_unmap (c2d_buf_outframe_);
    g_slice_free (GstVideoFrame, c2d_buf_outframe_);
    c2d_buf_outframe_ = nullptr;
  }
  if (nullptr != outpool_ &&
      nullptr != gst_c2d_buf_) {
    gst_buffer_pool_release_buffer (outpool_, gst_c2d_buf_);
    gst_buffer_pool_set_active (outpool_, FALSE);
    gst_object_unref (outpool_);
    gst_c2d_buf_ = nullptr;
    outpool_ = nullptr;
  }
  if (nullptr != buffers_.scale_buf && !use_c2d_preprocess_) {
    free(buffers_.scale_buf);
    buffers_.scale_buf = nullptr;
  }
}

void MLEngine::MeanSubtract(uint8_t* input_buf,
                            const uint32_t width,
                            const uint32_t height,
                            const uint32_t pad_width,
                            const uint32_t pad_height,
                            float* processed_buf) {

  MLE_UNUSED(pad_height);
  uint8_t* src = input_buf;
  float* dest = processed_buf;

  float std_blue = config_.use_norm ? config_.blue_sigma : 1;
  float std_green = config_.use_norm ? config_.green_sigma : 1;
  float std_red = config_.use_norm ? config_.red_sigma : 1;

  for (uint32_t y = 0; y < height; y++) {
    for (uint32_t x = 0; x < width; x++) {
      dest[((y * pad_width + x) * 3)] =
          (static_cast<float>(src[((y * width + x) * 3)]) -
              config_.red_mean) / std_red;

      dest[((y * pad_width + x) * 3) + 1] =
          (static_cast<float>(src[((y * width + x) * 3) + 1]) -
              config_.green_mean) / std_green;

      dest[((y * pad_width + x) * 3) + 2] =
          (static_cast<float>(src[((y * width + x) * 3) + 2]) -
              config_.blue_mean) / std_blue;
    }
  }
}

void MLEngine::Pad(
    uint8_t*       input_buf,
    const uint32_t input_width,
    const uint32_t input_height,
    const uint32_t pad_width,
    const uint32_t pad_height,
    uint8_t*       output_buf)
{
  MLE_UNUSED(pad_height);
  // This API assume that buffer is already fill up with
  // pad value and only active area is copied.
  // This optimization reduce time ~10 times.
  for (uint32_t y = 0; y < input_height; y++) {
    for (uint32_t x = 0; x < 3 * input_width; x++) {
      uint32_t index_src = y * 3 * input_width + x;
      uint32_t index_dst = y * 3 * pad_width + x;
      output_buf[index_dst] = input_buf[index_src];
    }
  }
}

int32_t MLEngine::PreProcessScale(
  uint8_t*       pSrcLuma,
  uint8_t*       pSrcChroma,
  uint8_t*       pDst,
  const uint32_t srcWidth,
  const uint32_t srcHeight,
  const uint32_t srcStride,
  const uint32_t scaleWidth,
  const uint32_t scaleHeight,
  MLEImageFormat format)
{
  MLE_LOGI("%s: Enter ", __func__);
  MLE_LOGV("%s: format %d preprocess_mode %d", __func__, format,
    (uint32_t)config_.preprocess_mode);

  int32_t rc = MLE_OK;
  if ((format == mle_format_nv12) || (format == mle_format_nv21)) {
    uint8_t *src_buffer_y = pSrcLuma;
    uint8_t *src_buffer_uv = pSrcChroma;
    uint32_t x = 0, y = 0;
    uint32_t width = srcWidth;
    uint32_t height = srcHeight;
    uint32_t src_y_offset  = 0;
    uint32_t src_uv_offset = 0;

    if (config_.preprocess_mode == kKeepARCrop) {
      double in_ar = 0, out_ar = 0;
      in_ar  = static_cast<double>(width) / height;
      out_ar = static_cast<double>(scaleWidth) / scaleHeight;

      if (in_ar > out_ar) {
        width = out_ar * height;
        x = (srcWidth - width) / 2;
        width = width & ~1;
        x = x & ~1;
      } else if (in_ar < out_ar) {
        height = width / out_ar;
        y = (srcHeight - height) / 2;
        height = height & ~1;
        y = y & ~1;
      }

      po_.width = width;
      po_.height = height;
      po_.x_offset = x;
      po_.y_offset = y;

      //Adjust the Y pointer.
      src_y_offset = y * srcWidth + x;
      //Adjust the UV pointer.
      src_uv_offset = (y/2) * srcWidth + x;

      src_buffer_y = reinterpret_cast<uint8_t *>
                        ((intptr_t)src_buffer_y + src_y_offset);
      src_buffer_uv = reinterpret_cast<uint8_t *>
                        ((intptr_t)src_buffer_uv + src_uv_offset);
    }

    MLE_LOGV("%s: Scale Luma plane", __func__);

    fcvScaleDownMNu8(src_buffer_y,
                     width,
                     height,
                     srcStride,
                     pDst,
                     scaleWidth,
                     scaleHeight,
                     0);

    MLE_LOGV("%s: Scale Croma plane", __func__);

    fcvScaleDownMNInterleaveu8(src_buffer_uv,
                     width/2,
                     height/2,
                     srcStride,
                     pDst + (scaleWidth*scaleHeight),
                     scaleWidth/2,
                     scaleHeight/2,
                     0);
  } else {
    MLE_LOGE("Unsupported format %d", (int)format);
    rc = MLE_IMG_FORMAT_NOT_SUPPORTED;
  }

  MLE_LOGI("%s: Exit", __func__);
  return rc;
}

void MLEngine::PreProcessColorConvertRGB(
    uint8_t*       pSrcLuma,
    uint8_t*       pSrcChroma,
    uint8_t*       pDst,
    const uint32_t width,
    const uint32_t height,
    MLEImageFormat format)
{
  if ((format == mle_format_nv12) || (format == mle_format_nv21)) {
    fcvColorYCbCr420PseudoPlanarToRGB888u8(pSrcLuma,
                                           pSrcChroma,
                                           width,
                                           height,
                                           0,
                                           0,
                                           pDst,
                                           0);
  }
}

void MLEngine::PreProcessColorConvertBGR(
    uint8_t*       pSrc,
    uint8_t*       pDst,
    const uint32_t width,
    const uint32_t height)
{
  fcvColorRGB888ToBGR888u8(pSrc,
                           width,
                           height,
                           0,
                           pDst,
                           0);
}

int32_t MLEngine::ReadLabelsFile(const std::string& file_name,
                      std::vector<std::string>& res,
                      size_t& found_label_count) {
  std::ifstream file(file_name);
  if (!file) {
    MLE_LOGE("%s: Labels file %s not found!", __func__, file_name.c_str());
    return MLE_FAIL;
  }
  res.clear();
  std::string line;
  while (std::getline(file, line)) {
    res.push_back(line);
  }
  found_label_count = res.size();
  const int padding = 16;
  while (res.size() % padding) {
    res.emplace_back();
  }
  return MLE_OK;
}

int32_t MLEngine::Init(const MLEInputParams* source_info) {
  MLE_LOGI("%s: Enter", __func__);
  int32_t res = MLE_OK;

  // Gather input configuration parameters
  source_params_.width  = source_info->width;
  source_params_.height = source_info->height;
  source_params_.format = source_info->format;

  // Load model from file
  std::string file_name = config_.model_file;
  res = LoadModel(file_name);
  if (MLE_OK != res) {
    MLE_LOGE("%s: Failed to load model from %s",
                __func__, file_name.c_str());
    return res;
  }
  MLE_LOGI("%s: Loaded model from %s", __func__, file_name.c_str());

  if (need_labels_) {
    file_name = config_.labels_file;
    res = ReadLabelsFile(file_name, labels_, label_count_);
    if (MLE_OK != res) {
      MLE_LOGE("%s: Failed to read labeles file %s",
                  __func__, file_name.c_str());
      return res;
    }
  }

  res = InitFramework();
  if (MLE_OK != res) {
    MLE_LOGE("%s: Failed to init framework", __func__);
    return res;
  }

  // Calculate downscale params
  if (config_.preprocess_mode == kKeepARPad) {
    float ratio = (engine_input_params_.width & ~0x1) * 1.0 /
                  fmax(source_params_.width, source_params_.height);
    scale_width_ = (uint32_t)(source_params_.width * ratio) & ~0x1;
    scale_height_ = (uint32_t)(source_params_.height * ratio) & ~0x1;
  } else {
    scale_width_ = engine_input_params_.width;
    scale_height_ = engine_input_params_.height;

    if (scale_width_ % 2 != 0 ||
        scale_height_ % 2 != 0) {
      MLE_LOGE("Error: Odd dimensions aren't supported for preprocess mode %d",
               (int)config_.preprocess_mode);
      return MLE_FAIL;
    }
  }

  MLE_LOGI("%s scale width %d scale height %d model width %d height %d",
           __func__,
           scale_width_,
           scale_height_,
           engine_input_params_.width,
           engine_input_params_.height);

  MLE_LOGD("%s nms_threshold %3f max_detection_result %d num_threads %d runtime %d label_count_ %zu",
           __func__,
           config_.nms_threshold,
           config_.max_detection_result,
           config_.number_of_threads,
           config_.runtime,
           label_count_
           );

  // Allocate internal buffers
  if (source_params_.width != engine_input_params_.width ||
      source_params_.height != engine_input_params_.height) {
    do_rescale_ = true;
  } else {
    do_rescale_ = false;
  }
  res = AllocateInternalBuffers();
  if (MLE_OK != res) {
    MLE_LOGE("%s Buffer allocation failed", __func__);
    return res;
  }

  if (use_c2d_preprocess_) {
    GstStructure *inopts = NULL;
    c2dconvert_ = gst_c2d_video_converter_new ();
    if (c2dconvert_) {
      GValue rects = G_VALUE_INIT, entry = G_VALUE_INIT, value = G_VALUE_INIT;

      inopts = gst_structure_new_empty ("options");

      g_value_init (&rects, GST_TYPE_ARRAY);
      g_value_init (&entry, GST_TYPE_ARRAY);
      g_value_init (&value, G_TYPE_INT);

      g_value_set_int (&value, 0);
      gst_value_array_append_value (&entry, &value);
      g_value_set_int (&value, 0);
      gst_value_array_append_value (&entry, &value);
      g_value_set_int (&value, source_params_.width);
      gst_value_array_append_value (&entry, &value);
      g_value_set_int (&value, source_params_.height);
      gst_value_array_append_value (&entry, &value);

      gst_value_array_append_value (&rects, &entry);
      g_value_reset (&entry);

      gst_structure_set_value (inopts,
          GST_C2D_VIDEO_CONVERTER_OPT_SRC_RECTANGLES, &rects);
      g_value_reset (&rects);

      g_value_set_int (&value, 0);
      gst_value_array_append_value (&entry, &value);
      g_value_set_int (&value, 0);
      gst_value_array_append_value (&entry, &value);
      g_value_set_int (&value, scale_width_);
      gst_value_array_append_value (&entry, &value);
      g_value_set_int (&value, scale_height_);
      gst_value_array_append_value (&entry, &value);

      gst_value_array_append_value (&rects, &entry);

      gst_structure_set_value (inopts,
          GST_C2D_VIDEO_CONVERTER_OPT_DEST_RECTANGLES, &rects);

      g_value_unset (&value);
      g_value_unset (&entry);
      g_value_unset (&rects);

      gst_c2d_video_converter_set_input_opts (c2dconvert_, 0, inopts);
    } else {
      MLE_LOGE("%s: Failed to create c2d converter", __func__);
      FreeInternalBuffers();
      res = MLE_FAIL;
    }
  }

  MLE_LOGI("%s: Exit", __func__);
  return res;
}

void MLEngine::Deinit(){
  MLE_LOGI("%s: Enter", __func__);
  if (use_c2d_preprocess_ && c2dconvert_)
    gst_c2d_video_converter_free (c2dconvert_);
  FreeInternalBuffers();
  MLE_LOGI("%s: Exit", __func__);
}

int32_t MLEngine::Process(GstVideoFrame *frame) {
  MLE_LOGI("%s: Enter", __func__);
  if (!frame || !frame->buffer) {
    MLE_LOGE("%s Null pointer!", __func__);
    return MLE_NULLPTR;
  }
  int32_t res = MLE_OK;

  {
    Timer t("Pre-process time");
    res = PreProcess(frame);
    if (MLE_OK != res) {
      MLE_LOGE(" PreProcessBuffer failed");
      return res;
    }
  }

  {
    Timer t("Inference time");
    res = ExecuteModel();
    if (MLE_OK != res) {
      MLE_LOGE(" Inference failed");
      return res;
    }
  }

  {
    Timer t("Post-process time");
    res = PostProcess(frame->buffer);
    if (MLE_OK != res) {
      MLE_LOGE(" PostProcess failed");
    }
  }

  MLE_LOGI("%s: Exit", __func__);
  return res;
}

int32_t MLEngine::PreProcess(GstVideoFrame *frame) {
  MLE_LOGI("%s: Enter", __func__);
  int32_t res = MLE_OK;

  uint8_t *frame_data_plane0 = (uint8_t*) GST_VIDEO_FRAME_PLANE_DATA (frame, 0);
  uint8_t *frame_data_plane1 = (uint8_t*) GST_VIDEO_FRAME_PLANE_DATA (frame, 1);

  void* engine_input_buf = GetInputBuffer();
  if (!engine_input_buf) {
    MLE_LOGE("%s: Input buffer is null", __func__);
    return MLE_NULLPTR;
  }

  if (use_c2d_preprocess_) {
    gpointer request_id = NULL;
    request_id = gst_c2d_video_converter_submit_request (c2dconvert_,
        frame, 1, c2d_buf_outframe_);
    gst_c2d_video_converter_wait_request (c2dconvert_, request_id);
  } else {
    if (do_rescale_) {
      uint32_t stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
      res = PreProcessScale(frame_data_plane0,
                            frame_data_plane1,
                            buffers_.scale_buf,
                            source_params_.width,
                            source_params_.height,
                            stride,
                            scale_width_,
                            scale_height_,
                            source_params_.format);
      if (MLE_OK != res) {
        MLE_LOGE("PreProcessScale failed due to unsupported image format");
        return res;
      }

      PreProcessColorConvertRGB(buffers_.scale_buf,
                                buffers_.scale_buf +
                                    (scale_width_ * scale_height_),
                                buffers_.rgb_buf,
                                scale_width_,
                                scale_height_,
                                source_params_.format);
    } else {
      PreProcessColorConvertRGB(frame_data_plane0,
                                frame_data_plane1,
                                buffers_.rgb_buf,
                                scale_width_,
                                scale_height_,
                                source_params_.format);
    }

    if (config_.input_format == kBgr ||
        config_.input_format == kBgrFloat) {
      PreProcessColorConvertBGR(buffers_.rgb_buf,
                                buffers_.rgb_buf,
                                scale_width_,
                                scale_height_);
    }
  }

  // MLE assumes mean subtract will be needed only if engine's input is float
  if (config_.input_format == kRgbFloat ||
      config_.input_format == kBgrFloat) {
    MeanSubtract(buffers_.rgb_buf,
                 scale_width_,
                 scale_height_,
                 engine_input_params_.width,
                 engine_input_params_.height,
                 (float*)engine_input_buf);
  } else {
    Pad(buffers_.rgb_buf,
        scale_width_,
        scale_height_,
        engine_input_params_.width,
        engine_input_params_.height,
        (uint8_t*)engine_input_buf);
  }

  MLE_LOGI("%s: Exit", __func__);
  return res;
}

GType
gst_mle_input_format_get_type (void)
{
  static GType gtype = 0;

  if (!gtype) {
    static const GEnumValue variants[] = {
      {kRgb, "RGB value as input", "RGB"},
      {kBgr, "BGR value as input", "BGR"},
      {kRgbFloat, "RGB float value as input", "RGB Float"},
      {kBgrFloat, "BGR float value as input", "BGR Float"},
      {0, NULL, NULL},
    };

    gtype = g_enum_register_static ("InputFormat", variants);
  }

  return gtype;
}

GType
gst_mle_preprocessing_mode_get_type (void)
{
  static GType gtype = 0;

  if (!gtype) {
    static const GEnumValue variants[] = {
      {kKeepARCrop, "keep aspect ratio by cropping", "Keep AR Crop"},
      {kKeepARPad, "keep aspect ratio by adding pads", "Keep AR Pad"},
      {kDirectDownscale, "directly down scale the frame", "Direct Downscale"},
      {kMax, "use max width or height", "Max"},
      {0, NULL, NULL},
    };

    gtype = g_enum_register_static ("PreprocessingMode", variants);
  }

  return gtype;
}

GType
gst_mle_snpe_runtime_type_get_type (void)
{
  static GType gtype = 0;

  if (!gtype) {
    static const GEnumValue variants[] = {
      {kSnpeCpu, "use CPU runtime", "CPU"},
      {kSnpeDsp, "use DSP runtime", "DSP"},
      {kSnpeGpu, "use GPU runtime", "GPU"},
      {kSnpeAip, "use API runtime", "AIP"},
      {0, NULL, NULL},
    };

    gtype = g_enum_register_static ("Runtime", variants);
  }

  return gtype;
}

GType
gst_mle_preprocessing_accel_get_type (void)
{
  static GType gtype = 0;
  if (!gtype) {
    static const GEnumValue variants[] = {
      {kPreprocessCpu,
          "Execute with FastCV CPU performance option",
          "CPU"},
      {kPreprocessDsp,
          "Execute with FastCV performance option",
          "DSP"},
      {kPreprocessGpu,
          "Execute with C2D preprocess",
          "GPU"},
      {0, NULL, NULL},
    };

    gtype = g_enum_register_static ("PreprocessAccel", variants);
  }

  return gtype;
}

GType
gst_mle_tflite_delegate_type_get_type (void)
{
  static GType gtype = 0;
  if (!gtype) {
    static const GEnumValue variants[] = {
      {kTfliteNnapi, "NNAPI delegate with DSP accelerator", "NNAPI"},
      {kTfliteNnapiNpu, "NNAPI delegate with NPU accelerator", "NPU"},
      {kTfliteHexagonNn, "HEXAGON NN delegate", "Hexagon NN"},
      {kTfliteGpu, "GPU delegate", "GPU"},
      {kTfliteXnnpack, "XNN PACK delegate", "XNN Pack"},
      {kTfliteCpu, "CPU runtime", "CPU"},
      {0, NULL, NULL},
    };

    gtype = g_enum_register_static ("DelegateType", variants);
  }
  return gtype;
}

}; // namespace mle
