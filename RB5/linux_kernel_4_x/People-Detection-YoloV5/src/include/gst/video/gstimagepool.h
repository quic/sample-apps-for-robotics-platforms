/*
* Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
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

#ifndef __GST_IMAGE_POOL_H__
#define __GST_IMAGE_POOL_H__

#include <gst/video/video.h>
#include <gst/allocators/allocators.h>

G_BEGIN_DECLS

#define GST_TYPE_IMAGE_BUFFER_POOL \
  (gst_image_buffer_pool_get_type ())
#define GST_IMAGE_BUFFER_POOL(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST ((obj), GST_TYPE_IMAGE_BUFFER_POOL, \
      GstImageBufferPool))
#define GST_IMAGE_BUFFER_POOL_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST ((klass), GST_TYPE_IMAGE_BUFFER_POOL, \
      GstImageBufferPoolClass))
#define GST_IS_IMAGE_BUFFER_POOL(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE ((obj), GST_TYPE_IMAGE_BUFFER_POOL))
#define GST_IS_IMAGE_BUFFER_POOL_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE ((klass), GST_TYPE_IMAGE_BUFFER_POOL))
#define GST_IMAGE_BUFFER_POOL_CAST(obj) ((GstImageBufferPool*)(obj))

/**
 * GST_IMAGE_BUFFER_POOL_OPTION_UBWC_MODE:
 *
 * An option indicating that the allocated buffer must be UBWC.
 */
#define GST_IMAGE_BUFFER_POOL_OPTION_UBWC_MODE "GstBufferPoolOptionUBWCMode"

typedef struct _GstImageBufferPool GstImageBufferPool;
typedef struct _GstImageBufferPoolClass GstImageBufferPoolClass;
typedef struct _GstImageBufferPoolPrivate GstImageBufferPoolPrivate;

#define GST_IMAGE_BUFFER_POOL_TYPE_ION "GstBufferPoolTypeIonMemory"
#define GST_IMAGE_BUFFER_POOL_TYPE_GBM "GstBufferPoolTypeGbmMemory"

struct _GstImageBufferPool
{
  GstBufferPool parent;

  GstImageBufferPoolPrivate *priv;
};

struct _GstImageBufferPoolClass
{
  GstBufferPoolClass parent;
};

GType gst_image_buffer_pool_get_type (void);

/// Creates a buffer pool for managing video frames.
GstBufferPool * gst_image_buffer_pool_new (const gchar * type);

/// Retrieve current set video configuration.
const GstVideoInfo * gst_image_buffer_pool_get_info (GstBufferPool * pool);

G_END_DECLS

#endif /* __GST_IMAGE_POOL_H__ */
