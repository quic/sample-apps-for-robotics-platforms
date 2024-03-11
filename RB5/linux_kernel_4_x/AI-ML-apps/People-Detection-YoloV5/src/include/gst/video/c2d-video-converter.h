/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
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
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *     * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __GST_C2D_VIDEO_CONVERTER_H__
#define __GST_C2D_VIDEO_CONVERTER_H__

#include <gst/video/video.h>
#include <gst/allocators/allocators.h>

G_BEGIN_DECLS

typedef struct _GstC2dRequest GstC2dRequest;

/**
 * GST_C2D_VIDEO_CONVERTER_OPT_SRC_RECTANGLES
 *
 * #GST_TYPE_ARRAY: Array of source rectangles.
 * Default: NULL
 *
 * Not applicable for output.
 */
#define GST_C2D_VIDEO_CONVERTER_OPT_SRC_RECTANGLES \
    "GstC2dVideoConverter.source-rectangles"

/**
 * GST_C2D_VIDEO_CONVERTER_OPT_DEST_RECTANGLES
 *
 * #GST_TYPE_ARRAY: Array of destination rectangles.
 * Default: NULL
 *
 * Not applicable for output.
 */
#define GST_C2D_VIDEO_CONVERTER_OPT_DEST_RECTANGLES \
    "GstC2dVideoConverter.destination-rectangles"

/**
 * GST_C2D_VIDEO_CONVERTER_OPT_FLIP_HORIZONTAL:
 *
 * #G_TYPE_BOOLEAN, flip output horizontally
 * Default: FALSE
 *
 * Not applicable for output
 */
#define GST_C2D_VIDEO_CONVERTER_OPT_FLIP_HORIZONTAL \
    "GstC2dVideoConverter.flip-horizontal"

/**
 * GST_C2D_VIDEO_CONVERTER_OPT_FLIP_VERTICAL:
 *
 * #G_TYPE_BOOLEAN, flip output horizontally
 * Default: FALSE
 *
 * Not applicable for output
 */
#define GST_C2D_VIDEO_CONVERTER_OPT_FLIP_VERTICAL \
    "GstC2dVideoConverter.flip-vertical"

/**
 * GstC2dVideoRotate:
 * @GST_C2D_VIDEO_ROTATE_NONE: disable rotation of the output
 * @GST_C2D_VIDEO_ROTATE_90_CW: rotate output 90 degrees clockwise
 * @GST_C2D_VIDEO_ROTATE_90_CCW: rotate output 90 degrees counter-clockwise
 * @GST_C2D_VIDEO_ROTATE_180: rotate output 180 degrees
 *
 * Different output rotation modes
 */
typedef enum {
  GST_C2D_VIDEO_ROTATE_NONE,
  GST_C2D_VIDEO_ROTATE_90_CW,
  GST_C2D_VIDEO_ROTATE_90_CCW,
  GST_C2D_VIDEO_ROTATE_180,
} GstC2dVideoRotate;

GST_VIDEO_API GType gst_c2d_video_rotation_get_type (void);
#define GST_TYPE_C2D_VIDEO_ROTATION (gst_c2d_video_rotation_get_type())

/**
 * GST_C2D_VIDEO_CONVERTER_OPT_ROTATION:
 *
 * #GST_TYPE_C2D_VIDEO_ROTATION, set the output rotation flags
 * Default: #GST_C2D_VIDEO_ROTATE_NONE.
 *
 * Not applicable for output
 */
#define GST_C2D_VIDEO_CONVERTER_OPT_ROTATION \
    "GstC2dVideoConverter.rotation"

/**
 * GST_C2D_VIDEO_CONVERTER_OPT_ALPHA:
 *
 * #G_TYPE_DOUBLE, alpha channel occupancy
 * Default: 1.0
 *
 * Not applicable for output
 */
#define GST_C2D_VIDEO_CONVERTER_OPT_ALPHA \
    "GstC2dVideoConverter.alpha"

/**
 * GST_C2D_VIDEO_CONVERTER_OPT_BACKGROUND:
 *
 * #G_TYPE_UINT, background color
 * Default: 0x00000000
 *
 * Not applicable for input
 */
#define GST_C2D_VIDEO_CONVERTER_OPT_BACKGROUND \
    "GstC2dVideoConverter.background"

/**
 * GST_C2D_VIDEO_CONVERTER_OPT_UBWC_FORMAT:
 *
 * #G_TYPE_BOOLEAN, whether buffers have UBWC (Universal Bandwidth Compression)
 * Default: FALSE
 */
#define GST_C2D_VIDEO_CONVERTER_OPT_UBWC_FORMAT \
    "GstC2dVideoConverter.ubwc-format"

typedef struct _GstC2dVideoConverter GstC2dVideoConverter;

GST_VIDEO_API GstC2dVideoConverter *
gst_c2d_video_converter_new             (void);

GST_VIDEO_API void
gst_c2d_video_converter_free            (GstC2dVideoConverter *convert);

GST_VIDEO_API gboolean
gst_c2d_video_converter_set_input_opts  (GstC2dVideoConverter *convert,
                                         guint index, GstStructure *opts);

GST_VIDEO_API gboolean
gst_c2d_video_converter_set_output_opts (GstC2dVideoConverter *convert,
                                         GstStructure *opts);

GST_VIDEO_API gpointer
gst_c2d_video_converter_submit_request  (GstC2dVideoConverter *convert,
                                         const GstVideoFrame *inframes,
                                         guint n_inputs,
                                         GstVideoFrame *outframe);

GST_VIDEO_API gboolean
gst_c2d_video_converter_wait_request    (GstC2dVideoConverter *convert,
                                         gpointer request_id);

GST_VIDEO_API void
gst_c2d_video_converter_flush           (GstC2dVideoConverter *convert);

G_END_DECLS

#endif /* __GST_C2D_VIDEO_CONVERTER_H__ */
