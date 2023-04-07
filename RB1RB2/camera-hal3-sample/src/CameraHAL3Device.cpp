/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <stdio.h>
#include <sys/time.h>
#include <map>
#include "CameraHAL3Device.h"

using namespace android;

static camera_module_t*        cameraModule;
static int                     cameraCameraId;
static camera3_device_t*       cameraDevice;
static camera3_stream_configuration_t cameraStreamConfig;
static std::vector<camera3_stream_t*> streamList;

static BufferGroup*                 streamBufferGroup[MAX_STREAM];  // each one for each stream

static int32_t                      fpsRange[2];

static android::CameraMetadata      currentMeta;
static std::list<CameraMetadata>    settingMetaQueue;

static DeviceCallback*              deviceCallback;


camera_metadata_t*      cameraCharacteristics;
CameraStreamData*       cameraStreams[MAX_STREAM];


static const int  minJpegBufferSize = sizeof(camera3_jpeg_blob) + 1024 * 512;
static int estimateJpegBufferSize(uint32_t width, uint32_t height);


//
// Thread variables
//

static CameraThread*    threadRequest;
static CameraThread*    threadResult;

static pthread_mutex_t  pendingRequestMutex;
static pthread_cond_t   pendingRequestCondition;
static pthread_mutex_t  settingMetadataMutex;
static pthread_cond_t   resultProcessCondition;

typedef std::map<int, PendingRequest*> RequestMap;
typedef std::pair<int, PendingRequest*> RequestMapElement;
static RequestMap pendingRequestMap;

CameraMetadata        cameraMetaSetting;


static void* deviceDoCaptureResultProcess(void* data);
static void* deviceDoCaptureRequest(void* data);
static int deviceInvokeOneCaptureRequest(int* requestNumberOfEachStream,int* frameNumber);

static void ProcessCaptureResult(const camera3_callback_ops *callback, const camera3_capture_result *result);
static void Notify(const struct camera3_callback_ops *callback, const camera3_notify_msg_t *message);

camera3_callback_ops callbackOps = {&ProcessCaptureResult, &Notify, nullptr, nullptr};

//
// ===================================================+
//


void deviceInit(camera_module_t* module, int cameraId)
{
    cameraModule = module;
    cameraCameraId = cameraId;

    pthread_mutex_init(&pendingRequestMutex, NULL);
    pthread_mutex_init(&settingMetadataMutex, NULL);

    pthread_condattr_t condition_attr;
    pthread_condattr_init(&condition_attr);
    pthread_condattr_setclock(&condition_attr, CLOCK_MONOTONIC);
    pthread_cond_init(&pendingRequestCondition, &condition_attr);
    pthread_cond_init(&resultProcessCondition, &condition_attr);
    pthread_condattr_destroy(&condition_attr);

    threadRequest = NULL;
    threadResult = NULL;
    memset(&cameraStreamConfig, 0, sizeof(camera3_stream_configuration_t));

    struct camera_info info;
    cameraModule->get_camera_info(cameraCameraId, &info);
    cameraCharacteristics = (camera_metadata_t *)info.static_camera_characteristics;
}


camera_metadata_t* deviceGetCharacteristics()
{
    return cameraCharacteristics;
}

int deviceOpenCamera()
{
    int ret = 0;
    printf("open Camera: %d\n",cameraCameraId);
    struct camera_info cameraInfo;
    char cameraName[32] = {0};
    snprintf(cameraName, sizeof(cameraName), "%d", cameraCameraId);
    ret = cameraModule->common.methods->open(&cameraModule->common, cameraName, (hw_device_t**)(&cameraDevice));
    if (ret != 0) {
        fprintf(stderr, "open Camera failed\n");
        return ret;
    }

    ret = cameraDevice->ops->initialize(cameraDevice, &callbackOps);
    ret = cameraModule->get_camera_info(cameraCameraId, &cameraInfo);
    cameraCharacteristics = (camera_metadata_t *)cameraInfo.static_camera_characteristics;
    return ret;
}

void deviceStopStreams()
{
    pthread_mutex_lock(&threadRequest->mutex);
    CameraRequestMessage* requestMessage = new CameraRequestMessage();
    requestMessage->type = REQUEST_MESSAGE_STOP;
    requestMessage->value = 1;
    threadRequest->messageQueue.push_back(requestMessage);
    pthread_mutex_unlock(&threadRequest->mutex);

    pthread_join(threadRequest->thread,NULL);
    pthread_mutex_destroy(&threadRequest->mutex);
    delete threadRequest;
    threadRequest = NULL;

    pthread_mutex_lock(&threadResult->mutex);
    CameraResultProcessMessage* resultMessage = new CameraResultProcessMessage();
    resultMessage->stop = true;
    threadResult->messageQueue.push_back(resultMessage);
    pthread_cond_signal(&resultProcessCondition);
    pthread_mutex_unlock(&threadResult->mutex);

    pthread_join(threadResult->thread,NULL);

    pthread_mutex_lock(&pendingRequestMutex);
    int try_count = 0;
    while (!pendingRequestMap.empty() && try_count < 5) {
        struct timespec time_target;
        clock_gettime(CLOCK_MONOTONIC, &time_target);
        long extend = time_target.tv_nsec + 800000000;
        time_target.tv_sec += (extend / 1000000000);
        time_target.tv_nsec = extend % 1000000000;
        if (pthread_cond_timedwait(&pendingRequestCondition, &pendingRequestMutex, &time_target) != 0) {
            try_count++;
        }
        continue;
    }
    if (!pendingRequestMap.empty()) {
        pendingRequestMap.clear();
    }
    pthread_mutex_unlock(&pendingRequestMutex);

    pthread_mutex_destroy(&threadResult->mutex);
    delete threadResult;
    threadResult = NULL;

    pthread_mutex_destroy(&pendingRequestMutex);
    pthread_mutex_destroy(&settingMetadataMutex);
    pthread_cond_destroy(&pendingRequestCondition);
    pthread_cond_destroy(&resultProcessCondition);

    int stream_size = (int)streamList.size();
    currentMeta.clear();
    for (int i = 0; i < stream_size; i++) {
        delete streamList[i];
        streamList[i] = NULL;

        bufferDeleteBuffers(cameraStreams[i]->bufferGroup);
        cameraStreams[i]->bufferGroup = NULL;

        delete cameraStreams[i];
        cameraStreams[i] = NULL;
    }
    streamList.erase(streamList.begin(), streamList.begin() + streamList.size());
    memset(&cameraStreamConfig, 0, sizeof(camera3_stream_configuration_t));
}

void deviceCloseCamera()
{
    cameraDevice->common.close(&cameraDevice->common);
}


void devicePreAllocateStreams(std::vector<camera3_stream_t*> inputStreams)
{
    for (uint32_t i = 0; i < inputStreams.size(); i++) {
        BufferGroup* bufferGroup = new BufferGroup();
        int StreamBufferMax = inputStreams[i]->max_buffers;
        if (inputStreams[i]->format == HAL_PIXEL_FORMAT_BLOB) {
            int size = estimateJpegBufferSize(inputStreams[i]->width, inputStreams[i]->height);
            bufferAllocateBuffers(bufferGroup,
                                  StreamBufferMax,
                                  size,
                                  1,
                                  (int32_t)(inputStreams[i]->format),
                                  inputStreams[i]->usage);
        } else {
            bufferAllocateBuffers(bufferGroup,
                                  StreamBufferMax,
                                  inputStreams[i]->width,
                                  inputStreams[i]->height,
                                  (int32_t)(inputStreams[i]->format),
                                  inputStreams[i]->usage);
        }
        streamBufferGroup[i] = bufferGroup;
    }
}


void deviceSetCallBack(DeviceCallback* callback)
{
    if (callback) {
        deviceCallback = callback;
    }
}


void deviceSetFpsRange(int min, int max)
{
    fpsRange[0] = min;
    fpsRange[1] = max;
};

void deviceConfigureStreams(std::vector<camera3_stream_t*> inputStreams, int operationMode)
{
    cameraStreamConfig.num_streams = inputStreams.size();
    streamList.resize(cameraStreamConfig.num_streams);
    for (unsigned int i = 0; i < cameraStreamConfig.num_streams; i++) {
        camera3_stream_t* cameraStream = new camera3_stream_t();
        cameraStream->stream_type = inputStreams[i]->stream_type;
        cameraStream->width       = inputStreams[i]->width;
        cameraStream->height      = inputStreams[i]->height;
        cameraStream->format      = inputStreams[i]->format;
        cameraStream->data_space  = inputStreams[i]->data_space;
        cameraStream->usage       = inputStreams[i]->usage;
        cameraStream->rotation    = inputStreams[i]->rotation;
        cameraStream->max_buffers = inputStreams[i]->max_buffers;
        cameraStream->priv        = inputStreams[i]->priv;
        streamList[i] = cameraStream;

        CameraStreamData* newStream = new CameraStreamData();
        cameraStreams[i] = newStream;
        newStream->streamId = i;

        cameraStreams[i]->bufferGroup = streamBufferGroup[i];
    }

    cameraStreamConfig.operation_mode = operationMode;
    cameraStreamConfig.streams = streamList.data();

    int ret = 0;
    ret = cameraDevice->ops->configure_streams(cameraDevice, &cameraStreamConfig);
    if (ret != 0) {
        fprintf(stderr, "configure_streams Error res:%d\n",ret);
        return;
    }
}

void deviceSetCurrentMeta(android::CameraMetadata* meta)
{
    pthread_mutex_lock(&settingMetadataMutex);
    currentMeta = *meta;
    settingMetaQueue.push_back(currentMeta);
    pthread_mutex_unlock(&settingMetadataMutex);

}

android::CameraMetadata* deviceGetCurrentMeta()
{
    return &currentMeta;
}

void deviceConstructDefaultRequestSettings(int index,camera3_request_template_t type, bool useDefaultMeta)
{
    cameraStreams[index]->metadata =
        (camera_metadata *)cameraDevice->ops->construct_default_request_settings(cameraDevice, type);
    cameraStreams[index]->streamType = type;
    if (useDefaultMeta) {
        pthread_mutex_lock(&settingMetadataMutex);
        camera_metadata *meta = clone_camera_metadata(cameraStreams[index]->metadata);
        currentMeta = meta;
        currentMeta.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fpsRange, 2);

        uint32_t tag_id;
        uint8_t pcr_enable = 0;
        sp<VendorTagDescriptor> vendorTags = android::VendorTagDescriptor::getGlobalVendorTagDescriptor();
        CameraMetadata::getTagFromName("org.quic.camera.EarlyPCRenable.EarlyPCRenable", vendorTags.get(), &tag_id);
        currentMeta.update(tag_id, &(pcr_enable), 1);
        settingMetaQueue.push_back(currentMeta);
        pthread_mutex_unlock(&settingMetadataMutex);
    }
}

int deviceGetValidAcceptedStreams(const StreamInfo *inputStream, std::vector<StreamInfo> &acceptedStreams)
{
    int ret = 0;

    if (cameraCharacteristics == nullptr || inputStream == nullptr) {
        return -1;
    }

    camera_metadata_ro_entry metadata_entry;
    ret = find_camera_metadata_ro_entry(cameraCharacteristics,
                                        ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
                                        &metadata_entry);
    if ((ret != 0) || ((metadata_entry.count % 4) != 0)) {
        return -1;
    }

    StreamInfo stream;
    for (int i = 0; i < (int)metadata_entry.count; i+=4) {
        if (metadata_entry.data.i32[i + 3] ==
            ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT) {
            if ((metadata_entry.data.i32[i] == inputStream->format) &&
                (metadata_entry.data.i32[i+1] == inputStream->width) &&
                (metadata_entry.data.i32[i+2] == inputStream->height )) {
                stream.width = metadata_entry.data.i32[i+1];
                stream.height = metadata_entry.data.i32[i+2];
                stream.format = metadata_entry.data.i32[i];
                acceptedStreams.push_back(stream);
            }
        }
    }
    return 0;
}


int deviceFindStream(camera3_stream_t* stream)
{
    for (int i = 0;i< (int)streamList.size(); i++) {
        if (streamList[i] == stream) {
            return i;
        }
    }
    return -1;
}

CameraStreamData* deviceGetStream(int index)
{
    return cameraStreams[index];
}


int deviceStartCaptureRequest(CameraThread* requestThread, CameraThread* resultThread)
{
    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_JOINABLE);

    pthread_mutex_init(&requestThread->mutex,NULL);
    pthread_create(&(requestThread->thread), &thread_attr,
                   deviceDoCaptureRequest, requestThread);
    threadRequest = requestThread;

    pthread_mutex_init(&resultThread->mutex,NULL);
    pthread_create(&(resultThread->thread), &thread_attr,
                   deviceDoCaptureResultProcess, resultThread);
    threadResult = resultThread;

    pthread_attr_destroy(&thread_attr);

    return 0;
}

CameraThread* deviceGetRequestThread()
{
    return threadRequest;
}

//
// ===============================================================
//


// Result Thread Loop
void* deviceDoCaptureResultProcess(void* threadData)
{
    CameraThread* cameraThread = (CameraThread*) threadData;

    while (true) {
        pthread_mutex_lock(&cameraThread->mutex);
        if (cameraThread->messageQueue.empty()) {
            struct timespec time_wait;
            clock_gettime(CLOCK_MONOTONIC, &time_wait);
            time_wait.tv_sec += 8;
            if(pthread_cond_timedwait(&resultProcessCondition, &cameraThread->mutex, &time_wait) != 0) {
                fprintf(stderr, "Result Process wait message timeout.\n");
                pthread_mutex_unlock(&cameraThread->mutex);
                continue;
            }
        }
        CameraResultProcessMessage* message = (CameraResultProcessMessage*)cameraThread->messageQueue.front();
        cameraThread->messageQueue.pop_front();
        if (message->stop == true) {
            cameraThread->stop = true;
            delete message;
            message = NULL;
            pthread_mutex_unlock(&cameraThread->mutex);
            return nullptr;
        }
        pthread_mutex_unlock(&cameraThread->mutex);
        camera3_capture_result result = message->result;
        result.output_buffers = message->streamBufferList.data();
        const camera3_stream_buffer_t* buffers = result.output_buffers;
        deviceCallback->capture_post_process(&result);

        for (int i = 0; i < (int)result.num_output_buffers; i++) {
            int index = deviceFindStream(buffers[i].stream);
            CameraStreamData* stream = cameraStreams[index];
            bufferPush(stream->bufferGroup, buffers[i].buffer);
        }

        message->streamBufferList.erase(
            message->streamBufferList.begin(),
            message->streamBufferList.begin() + message->streamBufferList.size());
        delete message;
        message = NULL;
    }
    return nullptr;
}

// Request Thread Loop
void* deviceDoCaptureRequest(void* threadData)
{
    CameraThread* cameraThread = (CameraThread*) threadData;
    bool snapshot = false;
    bool snapshot_stream = -1;

    while (true) {
        pthread_mutex_lock(&cameraThread->mutex);
        if (!(cameraThread->messageQueue.empty())) {
            CameraRequestMessage* message = (CameraRequestMessage*)cameraThread->messageQueue.front();
            cameraThread->messageQueue.pop_front();

            if (message->type == REQUEST_MESSAGE_STOP) {
                if (message->value == true) {
                    cameraThread->stop = true;
                    pthread_mutex_unlock(&cameraThread->mutex);
                    delete message;
                    message = nullptr;
                    return nullptr;
                }
            } else if (message->type == REQUEST_MESSAGE_SNAPSHOT) {
                snapshot = true;
                snapshot_stream = message->value;
                pthread_mutex_unlock(&cameraThread->mutex);
                delete message;
                message = nullptr;
            } else {
                pthread_mutex_unlock(&cameraThread->mutex);
                delete message;
                message = nullptr;
                continue;
            }
        }
        pthread_mutex_unlock(&cameraThread->mutex);

        int requestNumberOfOneStream[MAX_STREAM] = {0};
        for (int i = 0; i < MAX_STREAM; i++) {
            if (cameraThread->streamOn[i] ||
                (snapshot && snapshot_stream == i)) {
                requestNumberOfOneStream[i] = 1;
            }
        }

        int ret = deviceInvokeOneCaptureRequest(requestNumberOfOneStream, &(cameraThread->frameNumber));

        if (snapshot) {
            snapshot = false;
            snapshot_stream = -1;
        }

        if (ret != 0) {
            fprintf(stderr, "CaptureRequest ret=%d\n",ret);
            return nullptr;
        }
    }
    return nullptr;
}


int deviceInvokeOneCaptureRequest(int* requestNumberOfOneStream,int* frameNumber)
{
    pthread_mutex_lock(&pendingRequestMutex);
    if (pendingRequestMap.size() >= CAMERA_LIVING_REQUEST_MAX) {
        struct timespec time_target;
        clock_gettime(CLOCK_MONOTONIC,&time_target);
        time_target.tv_sec += 8;

        if (pthread_cond_timedwait(&pendingRequestCondition, &pendingRequestMutex, &time_target) != 0) {
            fprintf(stderr, "Pending Request wait timeout\n");
            pthread_mutex_unlock(&pendingRequestMutex);
            return -1;
        }
    }
    pthread_mutex_unlock(&pendingRequestMutex);

    PendingRequest* pendingRequest = new PendingRequest();
    CameraStreamData* stream = NULL;

    std::vector<camera3_stream_buffer_t> streamBufferList;
    for (int i = 0;i < (int)streamList.size();i++) {
        if (requestNumberOfOneStream[i] == 0) {
            continue;
        }
        stream = cameraStreams[i];
        camera3_stream_buffer_t stream_buffer;
        stream_buffer.buffer = (const native_handle_t**)(bufferPop(stream->bufferGroup));
        stream_buffer.stream = streamList[i];
        stream_buffer.status = 0;
        stream_buffer.release_fence = -1;
        stream_buffer.acquire_fence = -1;
        pendingRequest->request.num_output_buffers++;
        streamBufferList.push_back(stream_buffer);

    }
    pendingRequest->request.frame_number = *frameNumber;
    pendingRequest->request.settings = nullptr;
    pthread_mutex_lock(&settingMetadataMutex);
    if (!settingMetaQueue.empty()) {
        cameraMetaSetting = settingMetaQueue.front();
        settingMetaQueue.pop_front();
        pendingRequest->request.settings = (camera_metadata*) cameraMetaSetting.getAndLock();
    } else {
        pendingRequest->request.settings =  (camera_metadata*)cameraMetaSetting.getAndLock();
    }
    pthread_mutex_unlock(&settingMetadataMutex);
    pendingRequest->request.input_buffer = nullptr;
    pendingRequest->request.output_buffers = streamBufferList.data();

    pthread_mutex_lock(&pendingRequestMutex);
    pendingRequestMap.insert(RequestMapElement(*frameNumber, pendingRequest));
    pthread_mutex_unlock(&pendingRequestMutex);

    int ret = cameraDevice->ops->process_capture_request(cameraDevice, &(pendingRequest->request));
    if (ret != 0) {
        int index = 0;
        fprintf(stderr, "process_capture_quest failed, frame:%d", *frameNumber);
        for (int i = 0; i < (int)pendingRequest->request.num_output_buffers; i++) {
            index = deviceFindStream(streamBufferList[i].stream);
            stream = cameraStreams[index];
            bufferPush(stream->bufferGroup, streamBufferList[i].buffer);

        }
        pthread_mutex_lock(&pendingRequestMutex);
        pendingRequestMap.erase(*frameNumber);
        delete pendingRequest;
        pendingRequest = NULL;
        pthread_mutex_unlock(&pendingRequestMutex);
    } else {
        (*frameNumber)++;
        if (pendingRequest->request.settings != NULL) {
            cameraMetaSetting.unlock(pendingRequest->request.settings);
        }
    }

    return ret;
}

//
// ===============================================================
//

// Callback of the result of capture request

void ProcessCaptureResult(const camera3_callback_ops *, const camera3_capture_result *capture_result)
{
    if (capture_result->partial_result >= 1) {
        deviceCallback->handle_meta_data((camera3_capture_result *)capture_result);
    }

    if (capture_result->num_output_buffers > 0) {
        pthread_mutex_lock(&threadResult->mutex);
        if (!threadResult->stop) {
            CameraResultProcessMessage* message = new CameraResultProcessMessage();
            message->result = *capture_result;
            for (int i = 0; i < (int)capture_result->num_output_buffers; i++) {
                message->streamBufferList.push_back(capture_result->output_buffers[i]);
            }
            threadResult->messageQueue.push_back(message);
            pthread_cond_signal(&resultProcessCondition);
        }
        pthread_mutex_unlock(&threadResult->mutex);
    }
    pthread_mutex_lock(&pendingRequestMutex);
    RequestMap::iterator it = pendingRequestMap.find(capture_result->frame_number);
    if (it == pendingRequestMap.end()) {
        pthread_mutex_unlock(&pendingRequestMutex);
        return;
    }
    PendingRequest* pendingRequest = (PendingRequest*)it->second;
    pendingRequest->numberOfOutputBuffer += capture_result->num_output_buffers;
    pendingRequest->numberOfMetadata += capture_result->partial_result;
    if ((unsigned int)pendingRequest->numberOfOutputBuffer >= pendingRequest->request.num_output_buffers && pendingRequest->numberOfMetadata) {
        pendingRequestMap.erase(capture_result->frame_number);
        pthread_cond_signal(&pendingRequestCondition);
        delete pendingRequest;
        pendingRequest = NULL;
    }
    pthread_mutex_unlock(&pendingRequestMutex);
}

void Notify(const struct camera3_callback_ops *, const camera3_notify_msg_t *) {}


//
// ===============================================================
//

int estimateJpegBufferSize(uint32_t width, uint32_t height)
{

    int ret = 0;
    int maxJpegBufferSize = 0;
    camera_metadata_ro_entry jpegBufferMaxSize;
    ret = find_camera_metadata_ro_entry(cameraCharacteristics,
                                        ANDROID_JPEG_MAX_SIZE,
                                        &jpegBufferMaxSize);
    if (jpegBufferMaxSize.count == 0) {
        fprintf(stderr, "Find maximum JPEG size from metadat failed.!\n");
        return 0;
    }
    maxJpegBufferSize = jpegBufferMaxSize.data.i32[0];

    float scaleFactor = ((float)width * (float)height) /
        (((float)maxJpegBufferSize - (float)sizeof(camera3_jpeg_blob)) / 3.0f);
    int jpegBufferSize = minJpegBufferSize + (maxJpegBufferSize - minJpegBufferSize) * scaleFactor;

    return jpegBufferSize;
}

