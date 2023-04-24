/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include "CameraHAL3Buffer.h"

#include <condition_variable>
#include <mutex>

using namespace std;

static const char* ion_dev_file = "/dev/ion";

static int ionFd;
static std::mutex bufferMutex;
static std::condition_variable bufferConditionVar;


static int allocateOneBuffer(
        BufferGroup*       bufferGroup,
        unsigned int       index,
        unsigned int       width,
        unsigned int       height,
        unsigned int       format,
        unsigned long int  consumerFlags,
        buffer_handle_t*   pBuffer);


//
// =============================================================
//


void bufferDeleteBuffers(BufferGroup* bufferGroup)
{
    for (unsigned int i = 0; i < bufferGroup->totalBuffers; i++) {
        if (bufferGroup->buffers[i] != NULL) {
            munmap(bufferGroup->bufferBlocks[i].vaddress, bufferGroup->bufferBlocks[i].size);
            native_handle_close((native_handle_t *)bufferGroup->buffers[i]);
            native_handle_delete((native_handle_t *)bufferGroup->buffers[i]);

            bufferGroup->buffers[i] = NULL;
        }
    }
}

void bufferAllocateBuffers(
    BufferGroup *bufferGroup,
    unsigned int totalBuffers,
    unsigned int width,
    unsigned int height,
    unsigned int format,
    unsigned long int consumerFlags)
{
    for (uint32_t i = 0; i < totalBuffers; i++) {
        allocateOneBuffer(bufferGroup, i, width, height, format, consumerFlags, &bufferGroup->buffers[i]);
        bufferGroup->totalBuffers++;
        bufferGroup->freeBuffers.push_back(&bufferGroup->buffers[i]);
    }
}

void bufferPush(BufferGroup* bufferGroup, buffer_handle_t* buffer)
{
    unique_lock<mutex> lock(bufferMutex);
    bufferGroup->freeBuffers.push_back(buffer);
    bufferConditionVar.notify_all();
}

buffer_handle_t* bufferPop(BufferGroup* bufferGroup)
{
    unique_lock<mutex> lock(bufferMutex);
    if (bufferGroup->freeBuffers.size() == 0) {
        bufferConditionVar.wait(lock);
    }

    buffer_handle_t* buffer = bufferGroup->freeBuffers.front();
    bufferGroup->freeBuffers.pop_front();
    return buffer;
}

BufferBlock* bufferGetBufferInfo(BufferGroup* bufferGroup, buffer_handle_t* buffer)
{
    unique_lock<mutex> lock(bufferMutex);
    for (unsigned int i = 0;i < bufferGroup->totalBuffers;i++){
        if (*buffer == bufferGroup->buffers[i]){
            return &(bufferGroup->bufferBlocks[i]);
        }
    }
    return NULL;
}

//
// =============================================================
//

static int allocateOneBuffer(
        BufferGroup*       bufferGroup,
        unsigned int       index,
        unsigned int       width,
        unsigned int       height,
        unsigned int       format,
        unsigned long int  consumerFlags,
        buffer_handle_t*   pBuffer)
{
    int ret = 0;
    struct ion_allocation_data allocation_data;
    native_handle_t* native_handle = nullptr;
    size_t buffer_size;
    unsigned int stride = 0;
    unsigned int slice = 0;

    if (ionFd <= 0) {
        ionFd = open(ion_dev_file, O_RDONLY);
    }
    if (ionFd <= 0) {
        fprintf(stderr, "Ion dev file open failed. Error=%d\n", errno);
        return -EINVAL;
    }
    memset(&allocation_data, 0, sizeof(allocation_data));

    if (format == HAL_PIXEL_FORMAT_YCBCR_420_888 ||
         (consumerFlags & GRALLOC_USAGE_HW_COMPOSER) ||
         (consumerFlags & GRALLOC_USAGE_HW_TEXTURE) ||
         (consumerFlags & GRALLOC_USAGE_SW_WRITE_OFTEN)) {
        stride = ALIGN_BYTE(width, 512);
        slice = ALIGN_BYTE(height, 512);
        buffer_size = (size_t)(stride * slice * 3 / 2);
    } else { // if (format == HAL_PIXEL_FORMAT_BLOB)
        buffer_size = (size_t)(width);
    }

    allocation_data.len = ((size_t)(buffer_size) + 4095U) & (~4095U);

    allocation_data.flags = 1;
    //allocation_data.heap_id_mask = (1U << ION_SYSTEM_HEAP_ID);
    allocation_data.heap_id_mask = ION_SYSTEM_HEAP_ID;
    ret = ioctl(ionFd, _IOWR('I', 0, struct ion_allocation_data), &allocation_data);
    if (ret < 0) {
        fprintf(stderr, "ION allocation failed. ret=%d Error=%d fd=%d\n", ret, errno, ionFd);
        return ret;
    }

    bufferGroup->bufferBlocks[index].vaddress  = mmap(NULL, allocation_data.len, PROT_READ  | PROT_WRITE, MAP_SHARED, allocation_data.fd, 0);
    bufferGroup->bufferBlocks[index].fd        = allocation_data.fd;
    bufferGroup->bufferBlocks[index].size      = allocation_data.len;
    bufferGroup->bufferBlocks[index].width     = width;
    bufferGroup->bufferBlocks[index].height    = height;
    bufferGroup->bufferBlocks[index].stride    = stride;
    bufferGroup->bufferBlocks[index].slice     = slice;
    bufferGroup->bufferBlocks[index].format    = format;
    bufferGroup->bufferBlocks[index].allocationData = allocation_data;

    native_handle = native_handle_create(1, 4);
    (native_handle)->data[0] = bufferGroup->bufferBlocks[index].fd;
    (native_handle)->data[1] = 0;
    (native_handle)->data[2] = 0;
    (native_handle)->data[3] = 0;
    (native_handle)->data[4] = allocation_data.len;
    (native_handle)->data[5] = 0;

    *pBuffer = native_handle;

    return ret;
}


