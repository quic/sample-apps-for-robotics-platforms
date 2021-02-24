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
#define dst_image_path "./image/dst.yuv"

#define src_image_width 640
#define src_image_height 480
#define src_bpp ( 1.5 )
#define dst_bpp ( 2 )

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

    int dst_image_width   = src_image_width;      // 640
    int dst_image_height  = src_image_height;    // 480
    int read_buffer_size  = src_image_width * src_image_height * src_bpp;
    int write_buffer_size = dst_image_width * dst_image_height * dst_bpp;

    uint8_t *read_buffer   = new uint8_t[read_buffer_size];
    uint32_t *write_buffer = new uint32_t[write_buffer_size];

    fread(read_buffer, sizeof(uint8_t), read_buffer_size, src);

    // color conversion
    fcvColorYUV420toRGB565u8( read_buffer, src_image_width, src_image_height, (uint32_t *)write_buffer);

    fwrite(write_buffer, 1, write_buffer_size, dst);

    delete []read_buffer;
    delete []write_buffer;
    fclose(src);
    fclose(dst);
    return 0;
}
