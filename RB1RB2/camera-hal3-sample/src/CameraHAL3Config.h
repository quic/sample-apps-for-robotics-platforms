/*
* Copyright (c) 20202Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#ifndef CAMX_HAL3_CONFIG_H
#define CAMX_HAL3_CONFIG_H

#include <camera/CameraMetadata.h>

#define CONFIG_AUTO_EXPOSURE            0
#define CONFIG_EXPOSURE_TIME            1
#define CONFIG_EXPOSURE_ISO             2
#define CONFIG_ANTIBANDING              3
#define CONFIG_COLOR_CORRECTION         4
#define CONFIG_AUTO_WHITE_BALANCE       5
#define CONFIG_WHITE_BALANCE_COLOR_TEMP 6
#define CONFIG_WHITE_BALANCE_GAIN       7
#define CONFIG_SNAPSHOT_ROTATION        8


typedef struct _StreamInfo {
    int width;
    int height;
    int format;
} StreamInfo;

struct CamxHAL3Config {
    int cameraId;
    StreamInfo previewStream;
    StreamInfo snapshotStream;
    StreamInfo videoStream;
    int         fpsRange[2];
};

void configUpdateMeta(android::CameraMetadata* meta, int cmd);
void configUpdateMeta(android::CameraMetadata* meta, int cmd, int value);
void configUpdateMeta(android::CameraMetadata* meta, int cmd, float value1, float value2, float value3);



#endif // CAMX_HAL3_CONFIG_H
