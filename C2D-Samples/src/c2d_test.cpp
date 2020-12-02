/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <ion/ion.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include "C2DColorConverter.h"

static int ion_fd_get(void)
{
    static int fd = -1;

    if (fd == -1) {
        fd = ion_open();
    }

    return fd;
}

static int alloc_ion_buffer(int *ion_fd, int size, void **addr)
{
    int ret;

    if (!ion_fd || !addr || size < 0) {
        printf("%s: Invalid argument!\n", __FUNCTION__);
        return -1;
    }

    ret = ion_alloc_fd(ion_fd_get(), size, 4096, 1 << 25, 1, ion_fd);
    if (ret < 0) {
        printf("%s: ion allocate failed!\n", __FUNCTION__);
        return -1;
    }

    *addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, *ion_fd, 0);
    if (*addr == MAP_FAILED) {
        printf("%s: mmap failed!\n", __FUNCTION__);
        return -1;
    }

    return 0;
}

static int c2d_demo_test(char color_tag, char *src_name, char *dst_name)
{
    int ret;
    int src_fd, dst_fd;
    int src_size, dst_size;
    void *src_base, *dst_base;
    size_t src_width, src_height, src_stride;
    size_t dst_width, dst_height;
    ColorConvertFormat src_fmt, dst_fmt;
    FILE *fp, *fq;
    int32_t rotate = 0;
    unsigned long file_size = 0;
    struct stat statbuf;

    if (color_tag == 1) {
        src_width  = 1920;
        src_height = 1088;
        src_fmt    = YCbCr420SP;

        dst_width  = 1920;
        dst_height = 1088;
        dst_fmt    = RGBA8888;

        printf("OK, the picture color format yuv_nv12-->RGBA8888\n");
    }

    if (color_tag == 0) {
        rotate = 90;
        src_width  = 1920;
        src_height = 1088;
        src_fmt    = YCbCr420SP;

        dst_width  = 1920 / 2;
        dst_height = 1088 / 2;
        dst_fmt    = YCbCr420SP;

        printf("OK, the picture rotate 180 , scale 1/2\n");
    }

    C2DColorConverter *c2dcc = new C2DColorConverter();
    src_stride = c2dcc->calcStride(src_fmt, src_width);
    src_size = c2dcc->calcSize(src_fmt, src_width, src_height);
    dst_size = c2dcc->calcSize(dst_fmt, dst_width, dst_height);

    printf("%s: stride = %ld, src size = %d, dst size = %d\n", __FUNCTION__, src_stride, src_size, dst_size);

    alloc_ion_buffer(&src_fd, src_size, &src_base);
    alloc_ion_buffer(&dst_fd, dst_size, &dst_base);

    memset(dst_base, 0x00, dst_size);

    fp = fopen(src_name, "r");
    if(!fp) {
        printf("%s: src %s open failed!\n", __FUNCTION__, src_name);
        return -1;
    }

    stat(src_name, &statbuf);
    file_size = statbuf.st_size;
    if(file_size <= 0) {
        fclose(fp);
        return -1;
    }
 
    ret = fread(src_base, 1, file_size, fp);
    if (ret < 0) {
        printf("%s: fread failed!\n", __FUNCTION__);
        fclose(fp);
        return -1;
    }

    c2dcc->setRotation(rotate);

    c2dcc->setResolution(src_width, src_height, dst_width, dst_height, src_fmt, dst_fmt, 0, src_stride);

    c2dcc->convertC2D(src_fd, src_base, src_base, dst_fd, dst_base, dst_base);

    fq = fopen(dst_name, "w");
    if(!fq) {
        printf("%s: dst %s open failed!\n", __FUNCTION__, dst_name);
        fclose(fp);
        return -1;
    }

    ret = fwrite(dst_base, 1, dst_size, fq);
    if(ret < 0) {
        printf("%s: fwrite failed!\n", __FUNCTION__);
        fclose(fp);
        fclose(fq);
        return -1;
    }

    fclose(fp);
    fclose(fq);

    return 0;
}

int main(int argc, char *argv[])
{
	int input_tag = 1;
	int color_tag = 0;
	char choose;

    while (input_tag) {
        printf("please make sure whether need color format yuv_nv12-->RGBA8888!!!\n");
        printf("choose y/n :");
        scanf("%s", &choose);
        if (choose == 'y') {
            color_tag = 1;
        }
        input_tag = 0;
    }
    printf("starting C2D Convert!\n");
    c2d_demo_test(color_tag, argv[1], argv[2]);
    printf("C2D Convert Done!\n");

    return 0;

}
