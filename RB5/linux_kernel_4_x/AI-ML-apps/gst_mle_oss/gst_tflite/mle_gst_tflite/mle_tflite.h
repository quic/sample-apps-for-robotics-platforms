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

#ifndef __GST_MLE_TFLITE_H__
#define __GST_MLE_TFLITE_H__

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>
#include <gst/allocators/allocators.h>
#include <ml-meta/ml_meta.h>
#include "mle_engine/ml_engine_intf.h"

G_BEGIN_DECLS

#define GST_TYPE_MLE_TFLITE \
  (gst_mle_tflite_get_type())
#define GST_MLE_TFLITE(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_MLE_TFLITE,GstMLETFLite))
#define GST_MLE_TFLITE_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_MLE_TFLITE,GstMLETFLiteClass))
#define GST_IS_MLE_TFLITE(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_MLE_TFLITE))
#define GST_IS_MLE_TFLITE_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_MLE_TFLITE))
#define GST_MLE_TFLITE_CAST(obj)       ((GstMLETFLite *)(obj))

typedef struct _GstMLETFLite      GstMLETFLite;
typedef struct _GstMLETFLiteClass GstMLETFLiteClass;

struct _GstMLETFLite {
  GstVideoFilter      parent;

  mle::MLEInputParams source_info;
  mle::SourceFrame source_frame;
  mle::MLEngine* engine;
  gboolean is_init;
  guint property_mask;

  gchar *config_location;
  gchar *model_filename;
  gchar *labels_filename;
  gchar *postprocessing;
  gchar *delegate;
  guint preprocessing_type;
  guint input_format;
  gfloat blue_mean;
  gfloat green_mean;
  gfloat red_mean;
  gfloat blue_sigma;
  gfloat green_sigma;
  gfloat red_sigma;
  gfloat conf_threshold;
  guint num_threads;
};

struct _GstMLETFLiteClass {
  GstVideoFilterClass parent;
};

G_GNUC_INTERNAL GType gst_mle_tflite_get_type(void);

G_END_DECLS

#endif // __GST_MLE_TFLITE_H__
