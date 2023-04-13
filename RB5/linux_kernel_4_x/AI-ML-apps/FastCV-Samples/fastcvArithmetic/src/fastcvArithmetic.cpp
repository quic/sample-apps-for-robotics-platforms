/*
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
#define dst_media_image_path "./image/dst_media.yuv"
#define dst_Gaussian_image_path "./image/dst_Gaussian.yuv"
#define dst_diff_image_path "./image/dst_diff.yuv"

#define src_image_width 640
#define src_image_height 480
#define src_bpp ( 1.5 )
#define dst_bpp ( 1.5 )

int
main()
{
    FILE *src = fopen(src_image_path, "rb");
    FILE *dst_media = fopen(dst_media_image_path, "wrb+");
    FILE *dst_Gaussian = fopen(dst_Gaussian_image_path, "wrb+");
    FILE *dst_diff = fopen(dst_diff_image_path, "wb+");

    if ( !src || !dst_media || !dst_Gaussian)
    {
        fprintf(stderr, "open image file failed!\n");
        return -1;
    }

    int dst_image_width   = src_image_width;      // 640
    int dst_image_height  = src_image_height;     // 480
    int read_buffer_size  = src_image_width * src_image_height;
    int write_buffer_size = dst_image_width * dst_image_height;

    uint8_t *read_buffer   = new uint8_t[read_buffer_size];
    uint8_t *write_buffer = new uint8_t[write_buffer_size];

    fread(read_buffer, sizeof(uint8_t), read_buffer_size, src);

    // media filter
    fcvFilterMedian3x3u8(read_buffer, src_image_width, src_image_height, write_buffer);
    fwrite(write_buffer, sizeof(uint8_t), write_buffer_size, dst_media);
    memset(write_buffer, 0, write_buffer_size);

    // Gaussian filter
    fcvFilterGaussian3x3u8(read_buffer, src_image_width, src_image_height, write_buffer, 1);
    fwrite(write_buffer, sizeof(uint8_t), write_buffer_size, dst_Gaussian);
    memset(write_buffer, 0, write_buffer_size);

    // compare media image and Gaussian image
    uint8_t *read_media_buffer   = new uint8_t[read_buffer_size];
    uint8_t *read_Gaussian_buffer   = new uint8_t[read_buffer_size];

    fread(read_media_buffer, sizeof(uint8_t), read_buffer_size, dst_media);
    fread(read_Gaussian_buffer, sizeof(uint8_t), read_buffer_size, dst_Gaussian);

    // compare
    fcvImageDiffu8(read_media_buffer, read_Gaussian_buffer, src_image_width, src_image_height, write_buffer);
    fwrite(write_buffer, sizeof(uint8_t), write_buffer_size, dst_diff);

    delete []read_buffer;
    delete []write_buffer;
    fclose(src);
    fclose(dst_media);
    fclose(dst_Gaussian);
    fclose(dst_diff);
    return 0;
}
