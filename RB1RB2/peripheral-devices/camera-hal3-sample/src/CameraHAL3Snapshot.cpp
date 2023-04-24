/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <utils/Timers.h>

#include "CameraHAL3Snapshot.h"

#define MAX_PREVIEW_STREAM_BUFFER   12
#define MAX_SNAPSHOT_STREAM_BUFFER  8
#define JPEG_DEFUALT_QUALITY        85
#define JPEG_BLOB_ID                0xFF

#define IDX_PREVIEW                  0
#define IDX_SNAPSHOT                 1

static camera_module_t*     cameraModule;
CamxHAL3Config*             streamConfig;

static int                  cameraId;

CameraMetadata*             metadataExt;
CameraStreamCallbacks       *streamCallbacks;


static void init(camera_module_t* module, CamxHAL3Config* config);
static void setCallbacks(CameraStreamCallbacks *cbs);
static void preInitStreams();
static int initSnapshotStreams();
static void start();

static void CapturePostProcess(camera3_capture_result *result);
static void HandleMetaData(camera3_capture_result *result);

DeviceCallback deviceCallback = {
    CapturePostProcess,
    HandleMetaData
};


//
// ===================================================+
//

int startPreview(camera_module_t* module, CamxHAL3Config* config, CameraStreamCallbacks *cbs)
{
    init(module, config);
    setCallbacks(cbs);

    preInitStreams();
    deviceOpenCamera();
    if (initSnapshotStreams() < 0) {
        return -1;
    }
    start();
    return 0;
}

void stopPreview() {
    deviceStopStreams();
    deviceCloseCamera();
}

void updateMetaData(CameraMetadata* meta)
{
    deviceSetCurrentMeta(meta);
}

CameraMetadata* getCurrentMeta()
{
    return deviceGetCurrentMeta();
}

//
// ================================================================
//

static void init(camera_module_t* module, CamxHAL3Config* config)
{
    metadataExt         = NULL;
    streamCallbacks     = NULL;

    if (module && config) {
        cameraModule   = module;
        streamConfig   = config;
        cameraId = config->cameraId;
        deviceInit(cameraModule, cameraId);
    } else {
        fprintf(stderr, "invalid parameters!");
    }
}


void snapshot()
{
    if (streamConfig->snapshotStream.format != HAL_PIXEL_FORMAT_BLOB) {
        fprintf(stderr, "snapshot format != HAL_PIXEL_FORMAT_BLOB\n");
        return;
    }

    CameraThread* requestThread = deviceGetRequestThread();
    pthread_mutex_lock(&requestThread->mutex);
    CameraRequestMessage* message = new CameraRequestMessage();
    memset(message, 0, sizeof(CameraRequestMessage));
    message->type = REQUEST_MESSAGE_SNAPSHOT;
    message->value = IDX_SNAPSHOT;
    requestThread->messageQueue.push_back(message);
    pthread_mutex_unlock(&requestThread->mutex);
}

static void setCallbacks(CameraStreamCallbacks *cbs)
{
    if (cbs) {
        streamCallbacks = cbs;
    }
}

static void preInitStreams()
{
    camera3_stream_t previewStreamData;
    camera3_stream_t snapshotStreamData;
    std::vector<camera3_stream_t*> streams;
    int stream_num = 2;

    previewStreamData.stream_type = CAMERA3_STREAM_OUTPUT;
    previewStreamData.width = streamConfig->previewStream.width;
    previewStreamData.height = streamConfig->previewStream.height;
    previewStreamData.format = streamConfig->previewStream.format;
    previewStreamData.data_space = HAL_DATASPACE_UNKNOWN;
    previewStreamData.usage = GRALLOC_USAGE_HW_COMPOSER | GRALLOC_USAGE_HW_TEXTURE;
    previewStreamData.rotation = 0;
    previewStreamData.max_buffers = MAX_PREVIEW_STREAM_BUFFER;
    previewStreamData.priv = 0;

    snapshotStreamData.stream_type = CAMERA3_STREAM_OUTPUT;
    snapshotStreamData.width = streamConfig->snapshotStream.width;
    snapshotStreamData.height = streamConfig->snapshotStream.height;
    snapshotStreamData.format = streamConfig->snapshotStream.format;
    snapshotStreamData.data_space = HAL_DATASPACE_V0_JFIF;
    snapshotStreamData.usage = GRALLOC_USAGE_SW_READ_OFTEN;
    snapshotStreamData.rotation = 0;
    snapshotStreamData.max_buffers = MAX_SNAPSHOT_STREAM_BUFFER;
    snapshotStreamData.priv = 0;

    streams.resize(stream_num);
    streams[IDX_PREVIEW] = &previewStreamData;
    streams[IDX_SNAPSHOT] = &snapshotStreamData;

    devicePreAllocateStreams(streams);
}

static void start()
{
    deviceSetCallBack(&deviceCallback);
    CameraThread* resultThread = new CameraThread();
    CameraThread* requestThread = new CameraThread();

    requestThread->streamOn[IDX_PREVIEW] = true;
    resultThread->streamOn[IDX_SNAPSHOT] = false;

    deviceStartCaptureRequest(requestThread,resultThread);
}

static int initSnapshotStreams()
{
    int result = 0;
    camera3_stream_t previewStreamData;
    camera3_stream_t snapshotStreamData;
    std::vector<camera3_stream_t*> streams;
    int stream_num = 2;

    std::vector<StreamInfo> acceptedPreviewStreams;
    std::vector<StreamInfo> acceptedSnapshotStreams;

    StreamInfo previewTry = {
        streamConfig->previewStream.width,
        streamConfig->previewStream.height,
        streamConfig->previewStream.format};

    result = deviceGetValidAcceptedStreams(&previewTry, acceptedPreviewStreams);

    if (result < 0 || acceptedPreviewStreams.size() == 0) {
        fprintf(stderr, "Find stream for preview failed. width: %d, height: %d, format: %d\n",
                        streamConfig->previewStream.width,
                        streamConfig->previewStream.height,
                        streamConfig->previewStream.format);
        return -1;
    }

    previewStreamData.stream_type = CAMERA3_STREAM_OUTPUT;
    previewStreamData.width = acceptedPreviewStreams[0].width;
    previewStreamData.height = acceptedPreviewStreams[0].height;
    previewStreamData.format = acceptedPreviewStreams[0].format;
    previewStreamData.data_space = HAL_DATASPACE_UNKNOWN;
    previewStreamData.usage = GRALLOC_USAGE_HW_COMPOSER | GRALLOC_USAGE_HW_TEXTURE;
    previewStreamData.rotation = 0;
    previewStreamData.max_buffers = 0;
    previewStreamData.priv = 0;

    StreamInfo snapshotTry = {
        streamConfig->snapshotStream.width,
        streamConfig->snapshotStream.height,
        streamConfig->snapshotStream.format};

    result = deviceGetValidAcceptedStreams(&snapshotTry, acceptedSnapshotStreams);

    if (result < 0 || acceptedSnapshotStreams.size() == 0) {
        fprintf(stderr, "Find stream for snapshot failed. width: %d, height: %d, format: %d\n",
                        streamConfig->snapshotStream.width,
                        streamConfig->snapshotStream.height,
                        streamConfig->snapshotStream.format);
        return -1;
    }

    snapshotStreamData.stream_type = CAMERA3_STREAM_OUTPUT;
    snapshotStreamData.width = acceptedSnapshotStreams[0].width;
    snapshotStreamData.height = acceptedSnapshotStreams[0].height;
    snapshotStreamData.format = acceptedSnapshotStreams[0].format;
    snapshotStreamData.data_space = HAL_DATASPACE_V0_JFIF;
    snapshotStreamData.usage = GRALLOC_USAGE_SW_READ_OFTEN;
    snapshotStreamData.rotation = 0;
    snapshotStreamData.max_buffers = 0;
    snapshotStreamData.priv = 0;

    streams.resize(stream_num);
    streams[IDX_PREVIEW] = &previewStreamData;
    streams[IDX_SNAPSHOT] = &snapshotStreamData;

    deviceSetFpsRange(streamConfig->fpsRange[0], streamConfig->fpsRange[1]);
    deviceConfigureStreams(streams);
    if (metadataExt) {
        deviceSetCurrentMeta(metadataExt);
        deviceConstructDefaultRequestSettings(IDX_PREVIEW, CAMERA3_TEMPLATE_PREVIEW);
    } else {
        deviceConstructDefaultRequestSettings(IDX_PREVIEW, CAMERA3_TEMPLATE_PREVIEW, true);
    }

    deviceConstructDefaultRequestSettings(IDX_SNAPSHOT, CAMERA3_TEMPLATE_STILL_CAPTURE);

    uint8_t jpegQuality     = JPEG_DEFUALT_QUALITY;
    uint8_t ZslEnable       = ANDROID_CONTROL_ENABLE_ZSL_TRUE;
    android::CameraMetadata* currentMeta = getCurrentMeta();

    (*currentMeta).update(ANDROID_JPEG_QUALITY, &(jpegQuality), sizeof(jpegQuality));
    (*currentMeta).update(ANDROID_CONTROL_ENABLE_ZSL, &(ZslEnable), 1);
    updateMetaData(currentMeta);

    acceptedPreviewStreams.erase(acceptedPreviewStreams.begin(),
        acceptedPreviewStreams.begin() + acceptedPreviewStreams.size());
    acceptedSnapshotStreams.erase(acceptedSnapshotStreams.begin(),
        acceptedSnapshotStreams.begin() + acceptedSnapshotStreams.size());

    return result;
}

//
// ================================================================
//

// Callback from Device

static void CapturePostProcess(camera3_capture_result *capture_result)
{
    const camera3_stream_buffer_t* buffers = NULL;
    buffers = capture_result->output_buffers;

    for (unsigned int i = 0; i < capture_result->num_output_buffers; i++) {

        int index = deviceFindStream(buffers[i].stream);
        CameraStreamData* stream = deviceGetStream(index);
        BufferBlock* info = bufferGetBufferInfo(stream->bufferGroup, buffers[i].buffer);

        if (stream->streamType == CAMERA3_TEMPLATE_PREVIEW) {
            if (streamCallbacks && streamCallbacks->preview_callback) {
                streamCallbacks->preview_callback(info, capture_result->frame_number);
            }
        } else if (stream->streamType == CAMERA3_TEMPLATE_STILL_CAPTURE) {
            if (streamCallbacks && streamCallbacks->snapshot_callback) {
                streamCallbacks->snapshot_callback(info, capture_result->frame_number);
            }
        }
    }
}

static void HandleMetaData(camera3_capture_result *)
{ }

//
// ================================================================
//

void SaveFrame(const char* path, BufferBlock* bufferBlockInfo)
{
    unsigned long int size  = bufferBlockInfo->size;
    unsigned int width      = bufferBlockInfo->width;
    unsigned int height     = bufferBlockInfo->height;
    unsigned int stride     = bufferBlockInfo->stride;
    unsigned int slice       = bufferBlockInfo->slice;
    unsigned long int format = bufferBlockInfo->format;

    unsigned char* src_data = (unsigned char*)bufferBlockInfo->vaddress;

    if (format == HAL_PIXEL_FORMAT_BLOB) {
        FILE* file_descriptor = fopen(path, "wb");
        struct Camera3JPEGBlob cameraJpegBlob;
        size_t jpegOffsetToEof = (size_t)size - (size_t)sizeof(cameraJpegBlob);
        unsigned char* jpegEndOfFile = &src_data[jpegOffsetToEof];
        memcpy(&cameraJpegBlob, jpegEndOfFile, sizeof(Camera3JPEGBlob));

        if (cameraJpegBlob.JPEGBlobId == JPEG_BLOB_ID) {
            fwrite(src_data, cameraJpegBlob.JPEGBlobSize, 1, file_descriptor);
        } else {
            fwrite(src_data, size, 1, file_descriptor);
        }

        fclose(file_descriptor);
    } else if (format == HAL_PIXEL_FORMAT_YCBCR_420_888) {
        int plane_number  = 2;
        int byte_per_pixel = 1;

        FILE* file_descriptor = fopen(path, "wb");
        for (int i = 1; i <= plane_number; i++) {
            for (unsigned int h = 0; h < height / i; h++) {
                fwrite(src_data, (width * byte_per_pixel), 1, file_descriptor);
                src_data += stride;
            }
            src_data += stride * (slice - height);
        }
        fclose(file_descriptor);
    } else {
        fprintf(stderr, "%s format not support.\n", __FUNCTION__);
    }
}
