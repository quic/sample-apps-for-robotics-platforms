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

#include <cstring>
#include <fstream>
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mle_snpe.h"
#include "mle_engine/snpe_base.h"
#include "mle_engine/snpe_detection.h"
#include "mle_engine/snpe_single_ssd.h"
#include "mle_engine/snpe_segmentation.h"

#define GST_CAT_DEFAULT mle_snpe_debug
GST_DEBUG_CATEGORY_STATIC (mle_snpe_debug);

#define gst_mle_snpe_parent_class parent_class
G_DEFINE_TYPE (GstMLESNPE, gst_mle_snpe, GST_TYPE_VIDEO_FILTER);

#define GST_ML_VIDEO_FORMATS "{ NV12, NV21 }"

#define DEFAULT_PROP_SNPE_INPUT_FORMAT 3 //kBgrFloat
#define DEFAULT_PROP_MLE_MEAN_VALUE 128.0
#define DEFAULT_PROP_MLE_SIGMA_VALUE 128.0
#define DEFAULT_PROP_SNPE_RUNTIME 1
#define DEFAULT_PROP_MLE_CONF_THRESHOLD 0.5
#define DEFAULT_PROP_MLE_PREPROCESSING_TYPE 1 //kKeepARPad
#define GST_MLE_UNUSED(var) ((void)var)

enum {
  PROP_0,
  PROP_MLE_PARSE_CONFIG,
  PROP_MLE_FRAMEWORK_TYPE,
  PROP_MLE_MODEL_FILENAME,
  PROP_MLE_LABELS_FILENAME,
  PROP_SNPE_INPUT_FORMAT,
  PROP_MLE_POSTPROCESSING,
  PROP_MLE_MEAN_VALUES,
  PROP_MLE_SIGMA_VALUES,
  PROP_SNPE_RUNTIME,
  PROP_SNPE_OUTPUT_LAYERS,
  PROP_MLE_PREPROCESSING_TYPE,
  PROP_MLE_CONF_THRESHOLD,
};


static GstStaticCaps gst_mle_snpe_format_caps =
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE (GST_ML_VIDEO_FORMATS) ";"
    GST_VIDEO_CAPS_MAKE_WITH_FEATURES ("ANY", GST_ML_VIDEO_FORMATS));

static void
gst_mle_set_property_mask(guint &mask, guint property_id)
{
  mask |= 1 << property_id;
}

static gboolean
gst_mle_check_is_set(guint &mask, guint property_id)
{
  return (mask & 1 << property_id) ? true:false;
}

static void
gst_mle_snpe_set_property(GObject *object, guint property_id,
                          const GValue *value, GParamSpec *pspec)
{
  GstMLESNPE *mle = GST_MLE_SNPE (object);

  GST_OBJECT_LOCK (mle);
  switch (property_id) {
    case PROP_MLE_PARSE_CONFIG:
      gst_mle_set_property_mask(mle->property_mask, property_id);
      mle->config_location = g_strdup(g_value_get_string (value));
      break;
    case PROP_MLE_PREPROCESSING_TYPE:
      gst_mle_set_property_mask(mle->property_mask, property_id);
      mle->preprocessing_type = g_value_get_uint (value);
      break;
    case PROP_MLE_MODEL_FILENAME:
      gst_mle_set_property_mask(mle->property_mask, property_id);
      mle->model_filename = g_strdup(g_value_get_string (value));
      break;
    case PROP_MLE_LABELS_FILENAME:
      gst_mle_set_property_mask(mle->property_mask, property_id);
      mle->labels_filename = g_strdup(g_value_get_string (value));
      break;
    case PROP_SNPE_INPUT_FORMAT:
      gst_mle_set_property_mask(mle->property_mask, property_id);
      mle->input_format = g_value_get_uint (value);
      break;
    case PROP_MLE_POSTPROCESSING:
      gst_mle_set_property_mask(mle->property_mask, property_id);
      mle->postprocessing = g_strdup(g_value_get_string (value));
      break;
    case PROP_MLE_MEAN_VALUES:
      gst_mle_set_property_mask(mle->property_mask, property_id);
      mle->blue_mean =
          g_value_get_double (gst_value_array_get_value (value, 0));
      mle->green_mean =
          g_value_get_double (gst_value_array_get_value (value, 1));
      mle->red_mean =
          g_value_get_double (gst_value_array_get_value (value, 2));
      break;
    case PROP_MLE_SIGMA_VALUES:
      gst_mle_set_property_mask(mle->property_mask, property_id);
      mle->blue_sigma =
          g_value_get_double (gst_value_array_get_value (value, 0));
      mle->green_sigma =
          g_value_get_double (gst_value_array_get_value (value, 1));
      mle->red_sigma =
          g_value_get_double (gst_value_array_get_value (value, 2));
      break;
    case PROP_SNPE_RUNTIME:
      gst_mle_set_property_mask(mle->property_mask, property_id);
      mle->runtime = g_value_get_uint (value);
      break;
    case PROP_SNPE_OUTPUT_LAYERS:
      gst_mle_set_property_mask(mle->property_mask, property_id);
      mle->output_layers = g_strdup(g_value_get_string (value));
      break;
    case PROP_MLE_CONF_THRESHOLD:
      gst_mle_set_property_mask(mle->property_mask, property_id);
      mle->conf_threshold = g_value_get_float (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (mle);
}

static void
gst_mle_snpe_get_property(GObject *object, guint property_id,
                          GValue *value, GParamSpec *pspec)
{
  GstMLESNPE *mle = GST_MLE_SNPE (object);

  GST_OBJECT_LOCK (mle);
  switch (property_id) {
    case PROP_MLE_PARSE_CONFIG:
      g_value_set_string (value, mle->config_location);
      break;
    case PROP_MLE_PREPROCESSING_TYPE:
      g_value_set_uint (value, mle->preprocessing_type);
      break;
    case PROP_MLE_MODEL_FILENAME:
      g_value_set_string (value, mle->model_filename);
      break;
    case PROP_MLE_LABELS_FILENAME:
      g_value_set_string (value, mle->labels_filename);
      break;
    case PROP_SNPE_INPUT_FORMAT:
      g_value_set_uint (value, mle->input_format);
      break;
    case PROP_MLE_POSTPROCESSING:
      g_value_set_string (value, mle->postprocessing);
      break;
    case PROP_MLE_MEAN_VALUES: {
      GValue val = G_VALUE_INIT;
      g_value_init (&val, G_TYPE_DOUBLE);
      g_value_set_double (&val, mle->blue_mean);
      gst_value_array_append_value (value, &val);
      g_value_set_double (&val, mle->green_mean);
      gst_value_array_append_value (value, &val);
      g_value_set_double (&val, mle->red_mean);
      gst_value_array_append_value (value, &val);
      break;
    }
    case PROP_MLE_SIGMA_VALUES: {
      GValue val = G_VALUE_INIT;
      g_value_init (&val, G_TYPE_DOUBLE);
      g_value_set_double (&val, mle->blue_sigma);
      gst_value_array_append_value (value, &val);
      g_value_set_double (&val, mle->green_sigma);
      gst_value_array_append_value (value, &val);
      g_value_set_double (&val, mle->red_sigma);
      gst_value_array_append_value (value, &val);
      break;
    }
    case PROP_SNPE_RUNTIME:
      g_value_set_uint (value, mle->runtime);
      break;
    case PROP_SNPE_OUTPUT_LAYERS:
      g_value_set_string (value, mle->output_layers);
      break;
    case PROP_MLE_CONF_THRESHOLD:
      g_value_set_float (value, mle->conf_threshold);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (mle);
}

static void
gst_mle_snpe_finalize(GObject * object)
{
  GstMLESNPE *mle = GST_MLE_SNPE (object);

  if (mle->engine) {
    mle->engine->Deinit();
    delete (mle->engine);
    mle->engine = nullptr;
  }
  if (mle->output_layers) {
    g_free(mle->output_layers);
  }
  if (mle->model_filename) {
    g_free(mle->model_filename);
  }
  if (mle->labels_filename) {
    g_free(mle->labels_filename);
  }
  if (mle->config_location) {
    g_free(mle->config_location);
  }
  if (mle->postprocessing) {
    g_free(mle->postprocessing);
  }

  G_OBJECT_CLASS(parent_class)->finalize(G_OBJECT(mle));
}

static GstCaps *
gst_mle_snpe_caps(void)
{
  static GstCaps *caps = NULL;
  static volatile gsize inited = 0;
  if (g_once_init_enter(&inited)) {
    caps = gst_static_caps_get(&gst_mle_snpe_format_caps);
    g_once_init_leave(&inited, 1);
  }
  return caps;
}

static GstPadTemplate *
gst_mle_src_template(void)
{
  return gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS,
      gst_mle_snpe_caps());
}

static GstPadTemplate *
gst_mle_sink_template (void)
{
  return gst_pad_template_new("sink", GST_PAD_SINK, GST_PAD_ALWAYS,
      gst_mle_snpe_caps ());
}

static gboolean
gst_mle_snpe_parse_config(gchar *config_location,
                          mle::MLConfig &configuration) {
  gboolean rc = FALSE;
  GstStructure *structure = NULL;

  GValue gvalue = G_VALUE_INIT;
  g_value_init (&gvalue, GST_TYPE_STRUCTURE);

  if (g_file_test (config_location, G_FILE_TEST_IS_REGULAR)) {
    gchar *contents = NULL;
    GError *error = NULL;

    if (!g_file_get_contents (config_location, &contents, NULL, &error)) {
      GST_WARNING ("Failed to get config file contents, error: %s!",
          GST_STR_NULL (error->message));
      g_clear_error (&error);
      return FALSE;
    }

    // Remove trailing space and replace new lines with a coma delimeter.
    contents = g_strstrip (contents);
    contents = g_strdelimit (contents, "\n", ',');

    rc = gst_value_deserialize (&gvalue, contents);
    g_free (contents);

    if (!rc) {
      GST_WARNING ("Failed to deserialize config file contents!");
      return rc;
    }
  } else if (!gst_value_deserialize (&gvalue, config_location)) {
    GST_WARNING ("Failed to deserialize the config!");
    return FALSE;
  }

  structure = GST_STRUCTURE (g_value_dup_boxed (&gvalue));
  g_value_unset (&gvalue);

  gint value = 0;
  gdouble dvalue = 0.0;
  gboolean bvalue = false;

  if (gst_structure_get_int (structure, "input_format", &value))
    configuration.input_format = (mle::InputFormat)value;

  if (gst_structure_get_double (structure, "BlueMean", &dvalue))
    configuration.blue_mean = dvalue;

  if (gst_structure_get_double (structure, "BlueSigma", &dvalue))
   configuration.blue_sigma = dvalue;

  if (gst_structure_get_double (structure, "GreenMean", &dvalue))
    configuration.green_mean = dvalue;

  if (gst_structure_get_double (structure, "GreenSigma", &dvalue))
    configuration.green_sigma = dvalue;

  if (gst_structure_get_double (structure, "RedMean", &dvalue))
    configuration.red_mean = dvalue;

  if (gst_structure_get_double (structure, "RedSigma", &dvalue))
    configuration.red_sigma = dvalue;

  if (gst_structure_get_boolean (structure, "UseNorm", &bvalue))
    configuration.use_norm = dvalue;

  if (gst_structure_get_int (structure, "preprocess_type", &value))
    configuration.preprocess_mode = (mle::PreprocessingMode)value;

  if (gst_structure_get_double (structure, "confidence_threshold", &dvalue))
    configuration.conf_threshold = dvalue;

  if (gst_structure_get_int (structure, "num_threads", &value))
    configuration.number_of_threads = value;

  if (gst_structure_get_int (structure, "runtime", &value))
    configuration.runtime = (mle::RuntimeType)value;


  configuration.model_file = gst_structure_get_string (structure, "model");
  configuration.labels_file = gst_structure_get_string (structure, "labels");

  const GValue *gtempvalue =
      gst_structure_get_value (structure, "output_layers");
  if (gtempvalue != NULL && G_VALUE_HOLDS (gtempvalue, GST_TYPE_ARRAY)) {
    guint num = 0;

    for (num = 0; num < gst_value_array_get_size (gtempvalue); num++) {
      const GValue *val = gst_value_array_get_value (gtempvalue, num);
      std::string str = g_value_get_string (val);
      configuration.output_layers.push_back(str);
    }
  }

  gst_structure_free (structure);

  return rc;
}

static void
gst_mle_print_config(GstMLESNPE *mle,
                     mle::MLConfig &configuration,
                     gchar *postprocessing)
{
  GST_DEBUG_OBJECT(mle, "==== Configuration Begin ====");
  GST_DEBUG_OBJECT(mle, "Model %s", configuration.model_file.c_str());
  GST_DEBUG_OBJECT(mle, "Labels %s", configuration.labels_file.c_str());
  GST_DEBUG_OBJECT(mle, "Pre-processing %d",
                   (gint)configuration.preprocess_mode);
  GST_DEBUG_OBJECT(mle, "Mean(B,G,R): %f, %f, %f", configuration.blue_mean,
                                                   configuration.green_mean,
                                                   configuration.red_mean);
  GST_DEBUG_OBJECT(mle, "Sigma(B,G,R): %f, %f, %f", configuration.blue_sigma,
                                                    configuration.green_sigma,
                                                    configuration.red_sigma);
  GST_DEBUG_OBJECT(mle, "Confidence threshold %f",
                   configuration.conf_threshold);
  GST_DEBUG_OBJECT(mle, "Input format %d", (gint)configuration.input_format);
  GST_DEBUG_OBJECT(mle, "Runtime %d", (gint)configuration.runtime);
  for (guint i = 0; i < configuration.output_layers.size(); i++) {
    GST_DEBUG_OBJECT(mle, "Output layers[%d] %s", i,
                    configuration.output_layers[i].c_str());
  }
  GST_DEBUG_OBJECT(mle, "Post-processing %s", postprocessing);
  GST_DEBUG_OBJECT(mle, "==== Configuration End ====");
}

static void
gst_mle_parse_snpe_layers(gchar *src, std::vector<std::string> &dst)
{
  gchar *pch;
  gchar *saveptr;

  if (src) {
    pch = strtok_r(src, " ,", &saveptr);
    while (pch != NULL) {
      dst.push_back(pch);
      pch = strtok_r(NULL, " ,", &saveptr);
    }
  }
}

static gboolean
gst_mle_create_engine(GstMLESNPE *mle) {
  gboolean rc = TRUE;
  gboolean parse = TRUE;

  // Configuration structure for MLE
  // The order of priority is: default values < configuration file < property
  mle::MLConfig configuration {};

  // Set default configuration values
  configuration.blue_mean = configuration.green_mean = configuration.red_mean =
      DEFAULT_PROP_MLE_MEAN_VALUE;
  configuration.blue_sigma = configuration.green_sigma =
      configuration.red_sigma = DEFAULT_PROP_MLE_SIGMA_VALUE;
  configuration.input_format = (mle::InputFormat)mle->input_format;
  configuration.use_norm = false;
  configuration.runtime = (mle::RuntimeType)mle->runtime;
  configuration.preprocess_mode =
      (mle::PreprocessingMode)mle->preprocessing_type;
  configuration.conf_threshold = DEFAULT_PROP_MLE_CONF_THRESHOLD;
  configuration.io_type = mle::NetworkIO::kUserBuffer;

  // Set configuration values from config file
  if (mle->config_location) {
    parse = gst_mle_snpe_parse_config(mle->config_location, configuration);
    if (FALSE == parse) {
      GST_DEBUG_OBJECT(mle, "Parsing configuration failed.");
    } else {
      GST_DEBUG_OBJECT(mle, "Parsing from file is successful!");
    }
  }

  // Set configuration values only if property is set
  if (gst_mle_check_is_set(mle->property_mask, PROP_MLE_MODEL_FILENAME)) {
    configuration.model_file = mle->model_filename;
  }
  if (gst_mle_check_is_set(mle->property_mask, PROP_MLE_LABELS_FILENAME)) {
    configuration.labels_file = mle->labels_filename;
  }
  if (gst_mle_check_is_set(mle->property_mask, PROP_MLE_CONF_THRESHOLD)) {
    configuration.conf_threshold = mle->conf_threshold;
  }
  if (gst_mle_check_is_set(mle->property_mask, PROP_SNPE_INPUT_FORMAT)) {
    configuration.input_format = (mle::InputFormat)mle->input_format;
  }
  if (gst_mle_check_is_set(mle->property_mask, PROP_MLE_MEAN_VALUES)) {
    configuration.blue_mean = mle->blue_mean;
    configuration.green_mean = mle->green_mean;
    configuration.red_mean = mle->red_mean;
  }
  if (gst_mle_check_is_set(mle->property_mask, PROP_MLE_SIGMA_VALUES)) {
    configuration.blue_sigma = mle->blue_sigma;
    configuration.green_sigma = mle->green_sigma;
    configuration.red_sigma = mle->red_sigma;

    //set normalization flag
    configuration.use_norm = true;
  }
  if (gst_mle_check_is_set(mle->property_mask, PROP_SNPE_RUNTIME)) {
    configuration.runtime = (mle::RuntimeType) mle->runtime;
  }
  if (gst_mle_check_is_set(mle->property_mask, PROP_MLE_PREPROCESSING_TYPE)) {
    configuration.preprocess_mode =
        (mle::PreprocessingMode)mle->preprocessing_type;
  }

  if (gst_mle_check_is_set(mle->property_mask, PROP_SNPE_OUTPUT_LAYERS)) {
    configuration.output_layers.clear();
  }
  gst_mle_parse_snpe_layers(mle->output_layers, configuration.output_layers);

  gst_mle_print_config(mle, configuration, mle->postprocessing);

  if (!g_strcmp0(mle->postprocessing, "classification")) {
    mle->engine = new mle::SNPEBase(configuration);
    if (nullptr == mle->engine) {
      GST_ERROR_OBJECT (mle, "Failed to create SNPE instance.");
      rc = FALSE;
    }
  } else if (!g_strcmp0(mle->postprocessing, "detection")) {
      mle->engine = new mle::SNPEDetection(configuration);
      if (nullptr == mle->engine) {
        GST_ERROR_OBJECT (mle, "Failed to create SNPE instance.");
        rc = FALSE;
      }
  } else if(!g_strcmp0(mle->postprocessing, "singlessd")) {
      mle->engine = new mle::SNPESingleSSD(configuration);
      if (nullptr == mle->engine) {
        GST_ERROR_OBJECT (mle, "Failed to create SNPE instance.");
        rc = FALSE;
      }
  } else if (!g_strcmp0(mle->postprocessing, "segmentation")) {
      mle->engine = new mle::SNPESegmentation(configuration);
      if (nullptr == mle->engine) {
        GST_ERROR_OBJECT (mle, "Failed to create SNPE instance.");
        rc = FALSE;
      }
  } else {
    GST_ERROR_OBJECT (mle, "Unsupported SNPE postprocessing.");
    rc = FALSE;
  }

  return rc;
}

static mle::MLEImageFormat
gst_mle_get_video_format(GstVideoFormat &format)
{
  mle::MLEImageFormat mle_format = mle::MLEImageFormat::mle_format_invalid;
  switch (format) {
    case GST_VIDEO_FORMAT_NV12:
      mle_format = mle::MLEImageFormat::mle_format_nv12;
      break;
    case GST_VIDEO_FORMAT_NV21:
      mle_format = mle::MLEImageFormat::mle_format_nv21;
      break;
    default:
      mle_format = mle::MLEImageFormat::mle_format_invalid;
  }
  return mle_format;
}

static gboolean
gst_mle_snpe_set_info(GstVideoFilter *filter, GstCaps *in,
                      GstVideoInfo *ininfo, GstCaps *out,
                      GstVideoInfo *outinfo)
{
  GST_MLE_UNUSED(in);
  GST_MLE_UNUSED(out);
  GST_MLE_UNUSED(outinfo);

  gboolean rc = TRUE;
  GstMLESNPE *mle = GST_MLE_SNPE (filter);
  GstVideoFormat video_format = GST_VIDEO_INFO_FORMAT(ininfo);

  if (mle->engine && mle->is_init) {
    if ((gint)mle->source_info.width != GST_VIDEO_INFO_WIDTH(ininfo) ||
        (gint)mle->source_info.height != GST_VIDEO_INFO_HEIGHT(ininfo) ||
        mle->source_info.format != gst_mle_get_video_format(video_format)) {
      GST_DEBUG_OBJECT(mle, "Reinitializing due to source change.");
      mle->engine->Deinit();
      delete (mle->engine);
      mle->engine = nullptr;
      mle->is_init = FALSE;
    } else {
      GST_DEBUG_OBJECT(mle, "Already initialized.");
      return TRUE;
    }
  }

  gst_base_transform_set_passthrough (GST_BASE_TRANSFORM (filter), FALSE);

  mle->source_info.width = GST_VIDEO_INFO_WIDTH(ininfo);
  mle->source_info.height = GST_VIDEO_INFO_HEIGHT(ininfo);
  mle->source_info.format = gst_mle_get_video_format(video_format);
  if (mle->source_info.format != mle::MLEImageFormat::mle_format_nv12 &&
      mle->source_info.format != mle::MLEImageFormat::mle_format_nv21) {
    GST_ERROR_OBJECT (mle, "Video format not supported %d", video_format);
    return FALSE;
  }

  rc = gst_mle_create_engine(mle);
  if (FALSE == rc) {
    GST_ERROR_OBJECT (mle, "Failed to create MLE instance.");
    return rc;
  }

  gint ret = mle->engine->Init(&mle->source_info);
  if (ret) {
    GST_ERROR_OBJECT (mle, "MLE init failed. Please check the SNPE version. Install v1.43.0 or lower");
    delete (mle->engine);
    mle->engine = nullptr;
    rc = FALSE;
  } else {
    GST_DEBUG_OBJECT (mle, "MLE instance created addr %p", mle->engine);
    mle->is_init = TRUE;
  }

  return rc;
}

static GstFlowReturn gst_mle_snpe_transform_frame_ip(GstVideoFilter * filter,
                                                     GstVideoFrame * frame)
{
  GstMLESNPE *mle = GST_MLE_SNPE (filter);

  mle->source_frame.frame_data[0] = (uint8_t*)GST_VIDEO_FRAME_PLANE_DATA (frame, 0);
  mle->source_frame.frame_data[1] = (uint8_t*)GST_VIDEO_FRAME_PLANE_DATA (frame, 1);
  mle->source_frame.stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);

  gint ret = mle->engine->Process(&mle->source_frame, frame->buffer);
  if (ret) {
    GST_ERROR_OBJECT (mle, "MLE Process failed.");
    return GST_FLOW_ERROR;
  }

  return GST_FLOW_OK;
}

static void
gst_mle_snpe_class_init (GstMLESNPEClass * klass)
{
  GObjectClass *gobject            = G_OBJECT_CLASS (klass);
  GstElementClass *element         = GST_ELEMENT_CLASS (klass);
  GstVideoFilterClass *filter      = GST_VIDEO_FILTER_CLASS (klass);

  gobject->set_property = GST_DEBUG_FUNCPTR(gst_mle_snpe_set_property);
  gobject->get_property = GST_DEBUG_FUNCPTR(gst_mle_snpe_get_property);
  gobject->finalize     = GST_DEBUG_FUNCPTR(gst_mle_snpe_finalize);

  g_object_class_install_property(
      gobject,
      PROP_MLE_PARSE_CONFIG,
      g_param_spec_string(
          "config",
          "Path to config file",
          "Path to config file. Eg.: /data/misc/camera/mle_snpe.config",
          NULL,
          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                   G_PARAM_STATIC_STRINGS )));

  g_object_class_install_property(
      gobject,
      PROP_MLE_MODEL_FILENAME,
      g_param_spec_string(
          "model",
          "Model file",
          "Path to model file. Eg.: /data/misc/camera/model.dlc",
          NULL,
          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                   G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject,
      PROP_MLE_LABELS_FILENAME,
      g_param_spec_string(
          "labels",
          "Labels filename",
          "Path to labels file. Eg.: /data/misc/camera/labels.txt",
          NULL,
          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                   G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject,
      PROP_SNPE_INPUT_FORMAT,
      g_param_spec_uint(
          "input-format",
          "SNPE input format",
          "0 - RGB; 1 - BGR; 2 - RGBFloat; 3 - BGRFloat",
          0,
          3,
          DEFAULT_PROP_SNPE_INPUT_FORMAT,
          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                   G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject,
      PROP_MLE_POSTPROCESSING,
      g_param_spec_string(
          "postprocessing",
          "Postprocessing",
          "Supported Postprocessing: classification; detection; singlessd;"
                                    " segmentation",
          NULL,
          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                   G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject, PROP_MLE_MEAN_VALUES,
      gst_param_spec_array ("mean", "Mean Subtraction",
          "Channel Mean Subtraction values ('<B, G, R>')",
          g_param_spec_double ("value", "Mean Value",
              "One of B, G or R value.", 0, 255,
            DEFAULT_PROP_MLE_MEAN_VALUE,
            static_cast<GParamFlags>(G_PARAM_READWRITE |
                                     G_PARAM_STATIC_STRINGS)),
            static_cast<GParamFlags>(G_PARAM_READWRITE |
                                     G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject, PROP_MLE_SIGMA_VALUES,
      gst_param_spec_array ("sigma", "Sigma values",
          "Channel divisor values ('<B, G, R>')",
          g_param_spec_double ("value", "Sigma Value",
              "One of B, G or R divisors value.", 0, 255,
            DEFAULT_PROP_MLE_SIGMA_VALUE,
            static_cast<GParamFlags>(G_PARAM_READWRITE |
                                     G_PARAM_STATIC_STRINGS)),
            static_cast<GParamFlags>(G_PARAM_READWRITE |
                                     G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject,
      PROP_SNPE_RUNTIME,
      g_param_spec_uint(
          "runtime",
          "SNPE Runtime",
          "0 - CPU; 1 - DSP; 2 - GPU; 3 - AIP",
          0,
          3,
          DEFAULT_PROP_SNPE_RUNTIME,
          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                   G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject,
      PROP_SNPE_OUTPUT_LAYERS,
      g_param_spec_string(
          "output-layers",
          "SNPE output layers",
          "Model output layers, comma separated."
          "Should be set if model have more than one output",
          NULL,
          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                   G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject,
      PROP_MLE_PREPROCESSING_TYPE,
      g_param_spec_uint(
          "preprocess-type",
          "Preprocess type",
          "Possible values: 0-kKeepARCrop, 1-kKeepARPad, 2-kDirectDownscale",
          0,
          2,
          DEFAULT_PROP_MLE_PREPROCESSING_TYPE,
          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                   G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject,
      PROP_MLE_CONF_THRESHOLD,
      g_param_spec_float(
          "confidence-threshold",
          "Confidence Threshold",
          "Confidence Threshold value",
          0.0,
          1.0,
          DEFAULT_PROP_MLE_CONF_THRESHOLD,
          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                   G_PARAM_STATIC_STRINGS)));

  gst_element_class_set_static_metadata(
      element, "MLE SNPE", "Execute SNPE NN models",
      "Pre-process, execute NN model, post-process", "QTI");

  gst_element_class_add_pad_template(element,
                                     gst_mle_sink_template());
  gst_element_class_add_pad_template(element,
                                     gst_mle_src_template());

  filter->set_info = GST_DEBUG_FUNCPTR (gst_mle_snpe_set_info);
  filter->transform_frame_ip =
      GST_DEBUG_FUNCPTR (gst_mle_snpe_transform_frame_ip);
}

static void
gst_mle_snpe_init (GstMLESNPE * mle)
{
  mle->engine = nullptr;
  mle->config_location = nullptr;
  mle->is_init = FALSE;
  mle->input_format = DEFAULT_PROP_SNPE_INPUT_FORMAT;
  mle->blue_mean = mle->green_mean = mle->red_mean =
      DEFAULT_PROP_MLE_MEAN_VALUE;
  mle->blue_sigma = mle->green_sigma = mle->red_sigma =
      DEFAULT_PROP_MLE_SIGMA_VALUE;
  mle->output_layers = nullptr;
  mle->runtime = DEFAULT_PROP_SNPE_RUNTIME;
  mle->preprocessing_type = DEFAULT_PROP_MLE_PREPROCESSING_TYPE;
  mle->conf_threshold = DEFAULT_PROP_MLE_CONF_THRESHOLD;

  GST_DEBUG_CATEGORY_INIT (mle_snpe_debug, "qtimlesnpe", 0,
      "QTI Machine Learning Engine");
}

static gboolean
plugin_init (GstPlugin * plugin)
{
  return gst_element_register (plugin, "qtimlesnpe", GST_RANK_PRIMARY,
                               GST_TYPE_MLE_SNPE);
}

GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    qtimlesnpe,
    "Machine Learning Engine SNPE",
    plugin_init,
    PACKAGE_VERSION,
    PACKAGE_LICENSE,
    PACKAGE_SUMMARY,
    PACKAGE_ORIGIN
)
