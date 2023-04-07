/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <stdio.h>
#include <dlfcn.h>
#include <sys/stat.h>
#include <iostream>
#include <string>
#include <mutex>
#include <condition_variable>

#include "CameraHAL3Config.h"
#include "CameraHAL3Snapshot.h"

#define CAPTURE_OUTPUT_DIR "CAPTURE"

const char* CameraHALLibraryFile = "/usr/lib/hw/camera.qcom.so";
static camera_module_t*     cameraModule;
static bool savePreviewNow = false;
static std::mutex saveFrameMutex;
static std::condition_variable saveFrameCondition;

// Callback from camera module
// Parameters:
//   const struct camera_module_callbacks* callbacks
//   int camera_id
//   int new_status
void camera_device_status_change(const struct camera_module_callbacks*,  int, int)
{}

/**
 * @brief
 *
 * @param callbacks
 * @param camera_id
 * @param new_status
 */
// Callback from camera module
// Parameters:
//   const struct camera_module_callbacks* callbacks
//   const char* camera_id
//   int new_status
void torch_mode_status_change(const struct camera_module_callbacks* , const char*, int)
{}


// Callback struct for camera module.
camera_module_callbacks_t camera_module_callbacks =
{
    camera_device_status_change,
    torch_mode_status_change
};


int initial_module()
{
    struct camera_info camera_info;
    vendor_tag_ops_t vendor_tag_ops;
    int result = 0;
    void *fd = NULL;

    cameraModule = NULL;

    fd = dlopen(CameraHALLibraryFile, RTLD_NOW);
    if (fd == NULL) {
        fprintf(stderr, "Load %s module failed.\n", CameraHALLibraryFile);
        result = -EINVAL;
        cameraModule = NULL;
        return result;
    }

    const char *hal_module_info_sym = HAL_MODULE_INFO_SYM_AS_STR;
    cameraModule = (camera_module_t *)dlsym(fd, hal_module_info_sym);
    if (cameraModule == NULL) {
        fprintf(stderr, "Load symbol %s failed.\n", hal_module_info_sym);
        result = -EINVAL;
        if (fd != NULL) {
            dlclose(fd);
            fd = NULL;
        }
        return result;
    }

    if (strcmp(CAMERA_HARDWARE_MODULE_ID, cameraModule->common.id) != 0) {
        fprintf(stderr, "Load id %s != camera_module_ptr->id=%s\n",
                CAMERA_HARDWARE_MODULE_ID, cameraModule->common.id);
        result = -EINVAL;
        if (fd != NULL) {
            dlclose(fd);
            fd = NULL;
        }
        return result;
    }

    cameraModule->common.dso = fd;
    printf("Camera HAL library loaded.\n");

    printf("Initialize cameras...\n");
    result = cameraModule->init();
    if (result != 0) {
        fprintf(stderr, "Init camera failed. ret=%d\n", result);
        return result;
    }

    if (cameraModule->get_vendor_tag_ops) {
        vendor_tag_ops = vendor_tag_ops_t();
        cameraModule->get_vendor_tag_ops(&vendor_tag_ops);

        sp<VendorTagDescriptor> vendor_tag_descriptor;
        result = VendorTagDescriptor::createDescriptorFromOps(&vendor_tag_ops, vendor_tag_descriptor);

        if (result != 0) {
            fprintf(stderr, "Generate descriptor from vendor tag operations failed. ret=%d\n", result);
            return result;
        }

        result = VendorTagDescriptor::setAsGlobalVendorTagDescriptor(vendor_tag_descriptor);

        if (result != 0) {
            fprintf(stderr, "Set vendor tag descriptor failed. ret=%d\n", result);
            return result;
        }
    }

    result = cameraModule->set_callbacks(&camera_module_callbacks);
    if (result != 0) {
        fprintf(stderr, "set_callbacks failed. Error=%d\n", result);
        return result;
    }

    printf("Cameras list:\n");
    int num_cameras = cameraModule->get_number_of_cameras();
    for (int i = 0; i < num_cameras; i++) {
        int ret = cameraModule->get_camera_info(i, &camera_info);
        if (ret != 0) {
            printf("Get info from camera %d failed. ret=%d\n", i, ret);
            return ret;
        } else {
            printf(" camera %d\n", i);
        }
    }

    return result;
}


// Callback from camera HAL preview
void preview_callback(BufferBlock* buffer, int frameNum)
{
    std::unique_lock<std::mutex> lock(saveFrameMutex);
    if (savePreviewNow) {
        char filePath[256];
        time_t now;
        time(&now);
        struct tm *timeValue = localtime(&now);
        snprintf(filePath, sizeof(filePath), "%s/preview_420_888_%dx%d_%4d%02d%02d_%02d%02d%02d.yuv",
            CAPTURE_OUTPUT_DIR,
            buffer->width,
            buffer->height,
            timeValue->tm_year + 1900,
            timeValue->tm_mon + 1,
            timeValue->tm_mday,
            timeValue->tm_hour,
            timeValue->tm_min,
            timeValue->tm_sec);
        ::SaveFrame(filePath, buffer);
        printf("Save preview frame %d to:\n %s\n", frameNum, filePath);
        savePreviewNow = false;
        saveFrameCondition.notify_all();
    }
}


// Callback from camera HAL snapshot
void snapshot_callback(BufferBlock* buffer, int frameNum)
{
    std::unique_lock<std::mutex> lock(saveFrameMutex);
    char filePath[256];
    time_t now;
    time(&now);
    struct tm *timeValue = localtime(&now);
    snprintf(filePath, sizeof(filePath), "%s/snapshot_%4d%02d%02d_%02d%02d%02d.jpg",
        CAPTURE_OUTPUT_DIR,
        timeValue->tm_year + 1900,
        timeValue->tm_mon + 1,
        timeValue->tm_mday,
        timeValue->tm_hour,
        timeValue->tm_min,
        timeValue->tm_sec);
    ::SaveFrame(filePath, buffer);
    printf("Save snapshot frame %d to:\n %s\n", frameNum, filePath);
    saveFrameCondition.notify_all();
}


// Callback struct for camera streams.
CameraStreamCallbacks cameraStreamCallbacks =
{
    preview_callback,
    snapshot_callback
};


static void dumpPreviewFrame()
{
    std::unique_lock<std::mutex> lock(saveFrameMutex);
    savePreviewNow = true;
}

static void waitSaveFrame()
{
    std::unique_lock<std::mutex> lock(saveFrameMutex);
    saveFrameCondition.wait(lock);
}

static void setExposure();
static void setAntibanding();
static void setColorCorrection();
static void setWhiteBalance();
static void setSnapshotRotation();
static int getlineToInt(std::string prompt);
static float getlineToFloat(std::string prompt);

int main(int argc, char *argv[])
{
    int ret;
    int camera_id, preview_width, preview_height, snapshot_width, snapshot_height;
    bool finished = false;
    std::string command;

    if (argc != 6) {
        fprintf(stderr, "Usage: %s [camera id] [preview width] [preview height] [snapshot width] [snapshot height]\n", argv[0]);
        return -1;
    }

    camera_id = atoi(argv[1]);
    if (camera_id < 0 || camera_id > 3) {
        fprintf(stderr, "camera id: 0-3\n");
        return -1;
    }

    preview_width = atoi(argv[2]);
    preview_height = atoi(argv[3]);
    snapshot_width = atoi(argv[4]);
    snapshot_height = atoi(argv[5]);

    ret = initial_module();
    if (ret != 0) {
        fprintf(stderr, "Initial camera HAL module failed\n");
        return -1;
    }

    ret = mkdir(CAPTURE_OUTPUT_DIR, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (ret != 0 && errno != EEXIST) {
        fprintf(stderr, "Create output dir %s/ failed. Error=%d\n", CAPTURE_OUTPUT_DIR, errno);
        return -1;
    }

    CamxHAL3Config* conf = new CamxHAL3Config();
    conf->cameraId = camera_id;
    conf->previewStream.width = preview_width;
    conf->previewStream.height = preview_height;
    conf->previewStream.format = HAL_PIXEL_FORMAT_YCBCR_420_888;
    conf->snapshotStream.width = snapshot_width;
    conf->snapshotStream.height = snapshot_height;
    conf->snapshotStream.format = HAL_PIXEL_FORMAT_BLOB;

    ret = ::startPreview(cameraModule, conf, &cameraStreamCallbacks);
    if (ret < 0) {
        fprintf(stderr, "exit\n");
        return -1;
    }

    printf("Press h for help.\n");
    while(finished == false) {
        printf("> ");
        std::getline(std::cin, command);
        if (command == "p") {
            dumpPreviewFrame();
            waitSaveFrame();
        } else if (command == "s") {
            ::snapshot();
            waitSaveFrame();
        } else if (command == "q") {
            finished = true;
        } else if (command == "e") {
            setExposure();
        } else if (command == "a") {
            setAntibanding();
        } else if (command == "c") {
            setColorCorrection();
        } else if (command == "w") {
            setWhiteBalance();
        } else if (command == "r") {
            setSnapshotRotation();
        } else {
            if (command != "h" && !command.empty()) {
                printf("Unknown command \'%s\'.\n", command.c_str());
            }
            printf("Press p: save preview frame.\n");
            printf("Press s: take a snapshot.\n");
            printf("Press r: set snapshot rotation.\n");
            printf("Press e: set exposure.\n");
            printf("Press w: set white balance.\n");
            printf("Press a: set antibanding.\n");
            printf("Press c: set color correction.\n");
            printf("Press q: quit.\n");
        }
    }

    ::stopPreview();

    if (cameraModule != NULL && cameraModule->common.dso != NULL) {
        dlclose(cameraModule->common.dso);
        cameraModule = NULL;
    }

    return 0;
}

static void setExposure()
{
    std::string command;

    while(true) {
        printf("Press 1: auto, 2: exposure time, 3: ISO, q: quit\n");
        printf(">> ");

        std::getline(std::cin, command);

        if (command == "1") {
            android::CameraMetadata *meta = ::getCurrentMeta();
            configUpdateMeta(meta, CONFIG_AUTO_EXPOSURE);
            ::updateMetaData(meta);
            break;
        } else if (command == "2") {
            int ms_time;
            ms_time = getlineToInt("exposure time(Ms): ");
            android::CameraMetadata *meta = ::getCurrentMeta();
            configUpdateMeta(meta, CONFIG_EXPOSURE_TIME, ms_time);
            ::updateMetaData(meta);
            break;
        } else if (command == "3") {
            int ms_time;
            std::string str_ms_time;
            ms_time = getlineToInt("exposure time(Ms): ");
            android::CameraMetadata *meta = ::getCurrentMeta();
            configUpdateMeta(meta, CONFIG_EXPOSURE_TIME, ms_time);

            int iso;
            iso = getlineToInt("ISO (100 200 400 800 1600): ");
            configUpdateMeta(meta, CONFIG_EXPOSURE_ISO, iso);
            ::updateMetaData(meta);
            break;
        } else if (command == "q") {
            break;
        } else {
            printf("Wrong input.\n");
        }
    }
}

static void setAntibanding()
{
    std::string command;
    int ab;

    while(true) {
        printf("Press 0: off, 1: 50hz, 2: 60hz, 3: auto, q: quit\n");
        printf(">> ");

        std::getline(std::cin, command);

        if (command == "0" || command == "1" || command == "2" || command == "3") {
            ab = std::stoi(command);
            android::CameraMetadata *meta = ::getCurrentMeta();
            configUpdateMeta(meta, CONFIG_ANTIBANDING, ab);
            ::updateMetaData(meta);
            break;
        } else if (command == "q") {
            break;
        } else {
            printf("Wrong input.\n");
        }
    }
}

static void setColorCorrection()
{
    std::string command;
    int cc;

    while(true) {
        printf("Press 0: Matrix, 1: Fast, 2: Hight Quality, q: quit\n");
        printf(">> ");

        std::getline(std::cin, command);

        if (command == "0" || command == "1" || command == "2") {
            cc = std::stoi(command);
            android::CameraMetadata *meta = ::getCurrentMeta();
            configUpdateMeta(meta, CONFIG_COLOR_CORRECTION, cc);
            ::updateMetaData(meta);
            break;
        } else if (command == "q") {
            break;
        } else {
            printf("Wrong input.\n");
        }
    }
}

static void setWhiteBalance()
{
    std::string command;

    while(true) {
        printf("Press 1: auto, 2: color temperature, 3: gain, q: quit\n");
        printf(">> ");

        std::getline(std::cin, command);

        if (command == "1") {
            android::CameraMetadata *meta = ::getCurrentMeta();
            configUpdateMeta(meta, CONFIG_AUTO_WHITE_BALANCE);
            ::updateMetaData(meta);
            break;
        } else if (command == "2") {
            int color_temp;
            std::string str_color_temp;
            while(true) {
                printf("Color temperature (1: high, 0: mid, -1: low): ");
                std::getline(std::cin, str_color_temp);
                if (str_color_temp == "1" || str_color_temp == "0" || str_color_temp == "-1") {
                    color_temp = std::stoi(str_color_temp);
                    android::CameraMetadata *meta = ::getCurrentMeta();
                    configUpdateMeta(meta, CONFIG_WHITE_BALANCE_COLOR_TEMP, color_temp);
                    ::updateMetaData(meta);
                    break;
                } else {
                    printf("Wrong input.\n");
                }
            }
            return;
        } else if (command == "3") {
            float gain_r, gain_g, gain_b;
            printf("Input R G B gain (for ex: 2.0, 1.0, 0.9)\n");
            gain_r = getlineToFloat("r gain: ");
            gain_g = getlineToFloat("g gain: ");
            gain_b = getlineToFloat("b gain: ");

            android::CameraMetadata *meta = ::getCurrentMeta();
            configUpdateMeta(meta, CONFIG_WHITE_BALANCE_GAIN, gain_r, gain_g, gain_b);
            ::updateMetaData(meta);
            break;
        } else if (command == "q") {
            break;
        } else {
            printf("Wrong input.\n");
        }
    }
}

static void setSnapshotRotation()
{
    std::string command;

    while (true) {
        printf("Select degree: 0: 0, 1: 90, 2: 180, 3: 270, q: quit\n");
        printf(">> ");

        std::getline(std::cin, command);
        if (command == "0" || command == "1" || command == "2" || command == "3" ) {
            int rot = 0;
            if (command == "1") {
                rot = 90;
            } else if (command == "2") {
                rot = 180;
            }  else if (command == "3") {
                rot = 270;
            }
            android::CameraMetadata *meta = ::getCurrentMeta();
            configUpdateMeta(meta, CONFIG_SNAPSHOT_ROTATION, rot);
            ::updateMetaData(meta);
            break;
        } else if (command == "q") {
            break;
        } else {
            printf("Wrong input.\n");
        }
    }
}


static int getlineToInt(std::string prompt)
{
    int result;
    std::string str;

    while(true) {
        printf("%s", prompt.c_str());
        std::getline(std::cin, str);
        try {
            result = std::stoi(str);
            return result;
        }  catch (const std::invalid_argument& err) {
            printf("cannot covert %s to int.\n", str.c_str());
            continue;
        }
    }
}

static float getlineToFloat(std::string prompt)
{
    float result;
    std::string str;

    while(true) {
        printf("%s", prompt.c_str());
        std::getline(std::cin, str);
        try {
            result = std::stof(str);
            return result;
        }  catch (const std::invalid_argument& err) {
            printf("cannot covert %s to float.\n", str.c_str());
            continue;
        }
    }
}
