/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#ifndef CAMXHAL3BUFFER_H
#define CAMXHAL3BUFFER_H

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <deque>
#include <linux/ion.h>
#include <linux/msm_ion.h>
#include "hardware/camera3.h"

#define  BUFFER_QUEUE_MAX_SIZE  256

#define ALIGN_BYTE(x, a) ((x % a == 0) ? x : x - (x % a) + a)

typedef struct _BufferBlock {
    void*             vaddress;
    unsigned long int size;
    unsigned int      width;
    unsigned int      height;
    unsigned int      stride;
    unsigned int      slice;
    unsigned int      fd;
    unsigned int      format;
    struct ion_allocation_data allocationData;
} BufferBlock;

typedef struct _BufferGroup {
    std::deque<buffer_handle_t*> freeBuffers;
    uint32_t            totalBuffers;
    buffer_handle_t     buffers[BUFFER_QUEUE_MAX_SIZE];
    BufferBlock         bufferBlocks[BUFFER_QUEUE_MAX_SIZE];
} BufferGroup;

extern void bufferAllocateBuffers(
    BufferGroup *bufferGroup,
    unsigned int totalBuffers,
    unsigned int width,
    unsigned int height,
    unsigned int format,
    unsigned long int consumerFlags);

extern void bufferDeleteBuffers(BufferGroup* buffer);

extern void bufferPush(BufferGroup* bufferGroup, buffer_handle_t* buffer);
extern buffer_handle_t* bufferPop(BufferGroup* bufferGroup);
extern BufferBlock* bufferGetBufferInfo(BufferGroup* bufferGroup, buffer_handle_t* buffer);

#endif // CAMXHAL3BUFFER_H
