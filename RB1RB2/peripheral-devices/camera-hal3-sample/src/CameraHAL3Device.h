/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#ifndef CAMX_HAL3_DEVICE_H
#define CAMX_HAL3_DEVICE_H

#include <hardware/camera_common.h>
#include <hardware/camera3.h>
#include <camera/CameraMetadata.h>
#include <camera/VendorTagDescriptor.h>
#include <vector>
#include <list>

#include "CameraHAL3Config.h"
#include "CameraHAL3Buffer.h"


using namespace android;


#define MAX_STREAM                3
#define UNLIMIT_REQUEST_NUMBER   -1
#define CAMERA_LIVING_REQUEST_MAX  5

typedef struct _DeviceCallback {
    void (* capture_post_process)(camera3_capture_result *result);
    void (* handle_meta_data)(camera3_capture_result *result) {};
} DeviceCallback;


typedef struct _CameraStreamData {
    BufferGroup* bufferGroup;
    camera_metadata* metadata;
    int streamId;
    int streamType;
    camera3_stream_buffer_t streamBuffer;
} CameraStreamData;

typedef struct _CameraThread {
    pthread_t thread;
    pthread_mutex_t mutex;
    std::list<void*> messageQueue;
    bool stop;
    int frameNumber;
    bool streamOn[MAX_STREAM];

    _CameraThread() {
        stop = false;
        frameNumber = 0;
        for (int i = 0; i < MAX_STREAM; i++) {
            streamOn[i] = false;
        }
    }
} CameraThread;


#define REQUEST_MESSAGE_STOP        0
#define REQUEST_MESSAGE_SNAPSHOT    1

typedef struct _CameraRequestMessage {
    int type;
    int value;
} CameraRequestMessage;

#define DEVICE_PUT_BUFFER_INTERNAL  0
#define DEVICE_PUT_BUFFER_EXTERNAL  1

typedef struct _CameraPostProcessMessage {
    camera3_capture_result result;
    std::vector<camera3_stream_buffer_t> streamBufferList;
    bool stop;
} CameraResultProcessMessage;

class PendingRequest {
public:
    PendingRequest() {
        numberOfOutputBuffer = 0;
        numberOfMetadata = 0;
        memset(&request, 0, sizeof(camera3_capture_request_t));
    };
    camera3_capture_request_t request;
    int numberOfOutputBuffer;
    int numberOfMetadata;
};

extern void deviceInit(camera_module_t* module, int cameraId);
extern int deviceOpenCamera();
extern void deviceCloseCamera();
extern void deviceStopStreams();
extern camera_metadata_t* deviceGetCharacteristics();
extern void devicePreAllocateStreams(std::vector<camera3_stream_t*> streams);
extern void deviceSetCallBack(DeviceCallback* callback);
extern int deviceGetValidAcceptedStreams(const StreamInfo *inputStream, std::vector<StreamInfo> &acceptedStreams);
extern void deviceSetFpsRange(int min, int max);
extern void deviceConfigureStreams(std::vector<camera3_stream_t*> streams, int opMode = CAMERA3_STREAM_CONFIGURATION_NORMAL_MODE);
extern void deviceSetCurrentMeta(android::CameraMetadata* meta);
extern android::CameraMetadata* deviceGetCurrentMeta();
extern void deviceConstructDefaultRequestSettings(int index,camera3_request_template_t type, bool useDefaultMeta = false);
extern int deviceFindStream(camera3_stream_t* stream);
extern CameraStreamData* deviceGetStream(int index);
extern int deviceStartCaptureRequest(CameraThread* requestThread, CameraThread* resultThread);
extern CameraThread* deviceGetRequestThread();


#endif // CAMX_HAL3_DEVICE_H
