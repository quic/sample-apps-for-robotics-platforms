/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <memory.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/sysinfo.h>

#define FILE_CPUINFO   "/proc/cpuinfo"
#define FILE_GPUINFO   "/sys/devices/platform/soc/soc:qcom,gpubw/devfreq/soc:qcom,gpubw/%s"
#define FILE_IOMEM     "/proc/iomem"
#define FILE_FB0       "/dev/fb0"
#define FILE_SUBNAME   "/sys/bus/msm_subsys/devices/subsys%d/name"
#define FILE_SUBSTATE  "/sys/bus/msm_subsys/devices/subsys%d/state"
#define FILE_HDMISTATE "/sys/class/drm/card0-DSI-1/status"
#define FILE_CAN_FLAGS "/sys/class/net/can0/flags"
#define GPU_GOVERNOR   "governor"
#define GPU_CUR_FREQ   "cur_freq"
#define GPU_MIN_FREQ   "min_freq"
#define GPU_MAX_FREQ   "max_freq"
#define NET_UP         (1)
#define NET_BROADCAST  (1 << 1)
#define NET_LOOPBACK   (1 << 3)
#define NET_RUNNING    (1 << 6)
#define NET_NOARP      (1 << 7)
#define NET_MASTER     (1 << 10)
#define NET_MULTICAST  (1 << 12)
#define CPUSTART_NUM 12

enum cpufreqtype{
    freq_cur,
    freq_min,
    freq_max,
};

struct io_mem{
    void* add_start;
    void* add_end;
    struct io_mem* next;
    char io_name[20];
};

static int getcpuinfo_int(const char* attr_name, int* attr_val)
{
    char read_buf[1024] = {0};
    char* c_attr_val = NULL;
    FILE* fp;
    fp = fopen(FILE_CPUINFO, "r");
    if (fp == NULL) {
        printf("Can not open file: %s\n", FILE_CPUINFO);

        return -11;
    }

    *attr_val = 0;

    while (fscanf(fp, "%[^\n]", read_buf) != EOF)
    {
        if (strstr(read_buf, attr_name)) {
            c_attr_val = read_buf + CPUSTART_NUM;
            if (!(sscanf(c_attr_val, "%d", attr_val))) {
                fclose(fp);
                return -1;
            }
        }
        fgetc(fp);
    }

    fclose(fp);

    if (c_attr_val == NULL)
        return -2;

    return 0;
}

static int getcpufreq(int core_num, enum cpufreqtype type)
{
    char cpu_freq_file[64] = {0};
    int freq_val;
    FILE* fp;

    if (type == freq_cur) {
        snprintf(cpu_freq_file, sizeof(cpu_freq_file),
            "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_cur_freq", core_num);
    } else if (type == freq_min) {
        snprintf(cpu_freq_file, sizeof(cpu_freq_file),
            "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_min_freq", core_num);
    } else {
        snprintf(cpu_freq_file, sizeof(cpu_freq_file),
            "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq", core_num);
    }

    fp = fopen(cpu_freq_file, "r");
    if (fp == NULL) {
        printf("Can not open file: %s\n", cpu_freq_file);

        return -11;
    }

    fscanf(fp, "%d", &freq_val);
    fclose(fp);

    return freq_val;
}

static int getcpumode(int core_num, char* cpu_mode)
{
    char cpu_governor_file[64] = {0};
    int freq_val;
    FILE* fp;

    snprintf(cpu_governor_file, sizeof(cpu_governor_file),
        "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_governor", core_num);

    fp = fopen(cpu_governor_file, "r");
    if (fp == NULL) {
        printf("Can not open file: %s\n", cpu_governor_file);

        return -11;
    }

    fscanf(fp, "%s", cpu_mode);
    fclose(fp);

    return 0;
}

static int getgpuinfo(const char* attr_name, char* attr_info)
{
    char gpu_info_file[128] = {0};
    FILE* fp;

    snprintf(gpu_info_file, sizeof(gpu_info_file), FILE_GPUINFO, attr_name);

    fp = fopen(gpu_info_file, "r");
    if (fp == NULL) {
        printf("Can not open file: %s\n", gpu_info_file);

        return -11;
    }

    fscanf(fp, "%s", attr_info);
    fclose(fp);

    return 0;
}

static void* get_io_map()
{
    char read_buf[64];
    char* map_name;
    struct io_mem* first_io = NULL;
    FILE* fp;
    fp = fopen(FILE_IOMEM, "r");
    if (fp == NULL) {
        printf("Can not open file: %s\n", FILE_IOMEM);

        return NULL;
    }

    first_io = (struct io_mem*)malloc(sizeof(struct io_mem));
    memset(first_io, 0, sizeof(struct io_mem));
    free(first_io);
    while (fscanf(fp, "%[^\n]", read_buf) != EOF)
    {
        map_name = read_buf;
        while (*map_name == ' ')
        {
            map_name++;
        }

        printf("%s\n", map_name);
        fgetc(fp);
    }

    fclose(fp);

    return NULL;
}

static int getscreeninfo(int* width, int* height, int* bpp)
{
    int fd;
    struct fb_var_screeninfo fb_var;

    fd = open(FILE_FB0, O_RDWR);

    if (fd < 0) {
        printf("Open File: %s error!\n", FILE_FB0);
        return -11;
    }

    ioctl(fd, FBIOGET_VSCREENINFO, &fb_var);
    close(fd);

    *width = fb_var.xres;
    *height = fb_var.yres;
    *bpp = fb_var.bits_per_pixel;

    return 0;
}

static int getsubsysname(int subsys_num, char* subsys_name)
{
    char sub_sys_file[123] = {0};
    FILE* fp;

    snprintf(sub_sys_file, sizeof(sub_sys_file), FILE_SUBNAME, subsys_num);

    fp = fopen(sub_sys_file, "r");
    if (fp == NULL) {
        printf("Can not open file: %s\n", sub_sys_file);

        return -11;
    }

    fscanf(fp, "%s", subsys_name);
    fclose(fp);

    return 0;
}

static int getsubsysstate(int subsys_num, char* subsys_state)
{
    char sub_sys_file[123] = {0};
    FILE* fp;

    snprintf(sub_sys_file, sizeof(sub_sys_file), FILE_SUBSTATE, subsys_num);

    fp = fopen(sub_sys_file, "r");
    if (fp == NULL) {
        printf("Can not open file: %s\n", sub_sys_file);

        return -11;
    }

    fscanf(fp, "%s", subsys_state);
    fclose(fp);

    return 0;
}

static int getHDMIstatus(char* status)
{
    FILE* fp;

    fp = fopen(FILE_HDMISTATE, "r");
    if (fp == NULL) {
        printf("Can not open file: %s\n", FILE_HDMISTATE);

        return -11;
    }

    fscanf(fp, "%s", status);
    fclose(fp);

    return 0;
}

static int getCANflags(int* flags)
{
    FILE* fp;

    fp = fopen(FILE_CAN_FLAGS, "r");
    if (fp == NULL) {
        printf("Can not open file: %s\n", FILE_CAN_FLAGS);

        return -11;
    }

    fscanf(fp, "%x", flags);
    fclose(fp);

    return 0;
}

int main()
{
    char hostname[60] = {0};
    char cpu_per_mode[16] = {0};
    char gpu_info[16] = {0};
    char sub_system_name[16] = {0};
    char sub_system_state[16] = {0};
    char HDMI_status[16] = {0};
    int cpu_cores;
    int can_flags;
    int screen_width, screen_height, screen_bpp;
    struct sysinfo si;

    int i;
    int ret;

    ret = gethostname(hostname, sizeof(hostname));
    if (ret == 0)
        printf("hostname: %s\n", hostname);

    printf("\n*Peripheral device information*\n");
    printf("USB device connection   :\n");
    system("lsusb");

    ret = getHDMIstatus(HDMI_status);
    if (ret == 0)
        printf("\nHDMI device connection  : %s\n", HDMI_status);
    else
        printf("HDMI get status error!\n");

    ret = getCANflags(&can_flags);
    if (ret == 0)
        printf("\nCAN device connection   : %s\n",
            (can_flags & NET_RUNNING) ? "connected" : "disconnected");
    else
        printf("Get CAN device error!\n");

    printf("\n*****Subsystem information*****\n");
    for (i = 0; i < 10; i++)
    {
        ret = getsubsysname(i, sub_system_name);
        if (ret == 0) {
            printf("%s :", sub_system_name);
            ret = getsubsysstate(i, sub_system_name);
            if (ret == 0) {
                printf(" %s\n", sub_system_name);
            } else {
                printf("get subsystem state error!\n");
            }
        } else {
            printf("get subsystem name error!\n");
        }
    }


    printf("\n********CPU information********\n");

    ret = getcpuinfo_int("processor", &cpu_cores);
    if (ret == 0)
        printf("CPU cores              : %d\n", cpu_cores + 1);
    else
        printf("CPU cores              : error: %d\n", ret);

    for (i = 0; i <= cpu_cores; i++)
    {
        printf("CPU%d current frequency : %d\n", i, getcpufreq(i, freq_cur));
        printf("CPU%d MIN frequency     : %d\n", i, getcpufreq(i, freq_min));
        printf("CPU%d MAX frequency     : %d\n", i, getcpufreq(i, freq_max));
        ret = getcpumode(cpu_cores, cpu_per_mode);
        if (ret == 0)
            printf("CPU performance mode   : %s\n", cpu_per_mode);
        else
            printf("CPU performance mode get error!\n");
    }

    printf("\n********GPU information********\n");

    ret = getgpuinfo(GPU_GOVERNOR, gpu_info);
    if (ret == 0)
        printf("GPU governors          : %s\n", gpu_info);
    else
        printf("GPU governors get error!\n");

    ret = getgpuinfo(GPU_CUR_FREQ, gpu_info);
    if (ret == 0)
        printf("GPU cureent frequency  : %s\n", gpu_info);
    else
        printf("GPU cureent frequency get error!\n");

    ret = getgpuinfo(GPU_MIN_FREQ, gpu_info);
    if (ret == 0)
        printf("GPU min frequency      : %s\n", gpu_info);
    else
        printf("GPU min frequency get error!\n");

    ret = getgpuinfo(GPU_MAX_FREQ, gpu_info);
    if (ret == 0)
        printf("GPU max frequency      : %s\n", gpu_info);
    else
        printf("GPU max frequency get error!\n");

    printf("\n******Display information******\n");

    ret = getscreeninfo(&screen_width, &screen_height, &screen_bpp);
    if (ret == 0)
        printf("Screen resolution      : %d*%d-%dbpp\n",
            screen_width, screen_height, screen_bpp);
    else
        printf("GPU max frequency get error!\n");

    printf("\n*******Memory information******\n");

    sysinfo(&si);

    printf("Total RAM              : %lu\n", si.totalram);
    printf("Available RAM          : %lu\n", si.freeram);

    printf("\n*******IO map information******\n");
    get_io_map();

    return 0;
}
