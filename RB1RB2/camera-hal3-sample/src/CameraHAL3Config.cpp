/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include "CameraHAL3Config.h"
#include <camera/VendorTagDescriptor.h>


void configUpdateMeta(android::CameraMetadata* meta, int cmd, int value)
{

    android::sp<android::VendorTagDescriptor> vendorTags = android::VendorTagDescriptor::getGlobalVendorTagDescriptor();

    switch (cmd) {
        case CONFIG_EXPOSURE_TIME: {
            signed long int ns_time = (signed long int)(value * 1000000L);
            unsigned char ae = 0;
            (*meta).update(ANDROID_CONTROL_AE_MODE, &ae, 1);
            (*meta).update(ANDROID_SENSOR_EXPOSURE_TIME, &ns_time, 1);
        }
        case CONFIG_EXPOSURE_ISO: {
            int iso = value;
            (*meta).update(ANDROID_SENSOR_SENSITIVITY, &iso, 1);
        }
        break;
        case CONFIG_ANTIBANDING: {
            unsigned char ab = (unsigned char)value;
            (*meta).update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &ab, 1);
        }
        break;
        case CONFIG_COLOR_CORRECTION: {
            unsigned char cc = (unsigned char)value;
            (*meta).update(ANDROID_COLOR_CORRECTION_MODE, &cc, 1);
        }
        break;
        case CONFIG_WHITE_BALANCE_COLOR_TEMP: {
            unsigned char awb = 0;
            (*meta).update(ANDROID_CONTROL_AWB_MODE, &awb, 1);

            unsigned int tag = 0;
            int mode = 1;
            android::CameraMetadata::getTagFromName("org.codeaurora.qcamera3.manualWB.partial_mwb_mode", vendorTags.get(), &tag);
            (*meta).update(tag, &mode, 1);


            int ct = value;
            android::CameraMetadata::getTagFromName("org.codeaurora.qcamera3.manualWB.color_temperature",vendorTags.get(), &tag);
            (*meta).update(tag, &ct, 1);
        }
        break;
        case CONFIG_SNAPSHOT_ROTATION: {
            int rotation = value;
            (*meta).update(ANDROID_JPEG_ORIENTATION, &rotation, 1);
        }
        break;
        default:
        break;
    }
}


void configUpdateMeta(android::CameraMetadata* meta, int cmd)
{
    switch (cmd) {
        case CONFIG_AUTO_EXPOSURE: {
            unsigned char ae = 1;
            (*meta).update(ANDROID_CONTROL_AE_MODE, &ae, 1);
        }
        break;
        case CONFIG_AUTO_WHITE_BALANCE: {
            unsigned char awb = 1;
            (*meta).update(ANDROID_CONTROL_AWB_MODE, &awb, 1);
        }
        break;
        default:
        break;
    }
}


void configUpdateMeta(android::CameraMetadata* meta, int cmd, float value1, float value2, float value3)
{
    android::sp<android::VendorTagDescriptor> vendorTags = android::VendorTagDescriptor::getGlobalVendorTagDescriptor();

    switch (cmd) {
        case CONFIG_WHITE_BALANCE_GAIN: {
            unsigned char awb = 0;
            (*meta).update(ANDROID_CONTROL_AWB_MODE, &awb, 1);

            unsigned int tag = 0;
            int mode = 2;
            android::CameraMetadata::getTagFromName("org.codeaurora.qcamera3.manualWB.partial_mwb_mode", vendorTags.get(), &tag);
            (*meta).update(tag, &mode, 1);

            float gains[3] = {value1, value2, value3};
            android::CameraMetadata::getTagFromName("org.codeaurora.qcamera3.manualWB.gains",vendorTags.get(), &tag);
            (*meta).update(tag, gains, 3);
        }
        break;
        default:
        break;
    }
}
