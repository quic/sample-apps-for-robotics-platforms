/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#ifndef CAMX_HAL3_SNAPSHOT_H
#define CAMX_HAL3_SNAPSHOT_H

#include <pthread.h>
#include <hardware/camera_common.h>


#include "CameraHAL3Buffer.h"
#include "CameraHAL3Config.h"
#include "CameraHAL3Device.h"

typedef enum {
    TYPE_PREVIEW   = 0,
    TYPE_SNAPSHOT  = 1,
} StreamType;


typedef struct _camera_stream_callbacks {
    void (*preview_callback)(BufferBlock* buffer, int frameNum);
    void (*snapshot_callback)(BufferBlock* buffer, int frameNum);
} CameraStreamCallbacks;

struct Camera3JPEGBlob {
    uint16_t JPEGBlobId;
    uint32_t JPEGBlobSize;
};

extern int startPreview(camera_module_t* module, CamxHAL3Config* config, CameraStreamCallbacks *cbs);
extern void stopPreview();
extern void SaveFrame(const char* path, BufferBlock* info);
extern void snapshot();

extern CameraMetadata* getCurrentMeta();
extern void updateMetaData(CameraMetadata* meta);


#endif // CAMX_HAL3_SNAPSHOT_H
