/*
 *
 * * Copyright (c) 2020 Qualcomm Innovation Center, Inc. All Rights Reserved.
 *
 * *
 *
 * * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <stdlib.h>
#include "fastcv.h"
#include <dlfcn.h>

#define src_image_path "./image/src.yuv"
#define dst_image_path "./image/dst.y"
#define src_image_width 640
#define src_image_height 480
#define src_image_size (src_image_width * src_image_height)

int
main()
{
    FILE *src = fopen(src_image_path, "rb");
    FILE *dst = fopen(dst_image_path, "wb+");

    if ( !src || !dst)
    {
        fprintf(stderr, "open image file failed!\n");
        return -1;
    }

    uint8_t dst_image_width = src_image_width / 2;
    uint8_t dst_image_height = src_image_height / 2;
    uint8_t dst_image_size = dst_image_width * dst_image_height;
    uint8_t *read_buffer = new uint8_t[src_image_size];
    uint8_t *write_buffer = new uint8_t[dst_image_size];
    fread(read_buffer, src_image_size, 1, src);

    // downscale
    fcvScaleDownBy2u8( (uint8_t *)read_buffer, src_image_width, src_image_height, (uint8_t *)write_buffer);

    fwrite(write_buffer, src_image_size, 1, dst);
    delete []read_buffer;
    delete []write_buffer;
    fclose(src);
    fclose(dst);
    return 0;
}
