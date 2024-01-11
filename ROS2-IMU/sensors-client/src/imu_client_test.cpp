/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <sys/socket.h>
#include <glib.h>
#include <signal.h>
#include "imu_client.hpp"

ImuClient _imu_client;

void handler(int sig)
{
    if (!_imu_client.SendMsgStop(GYRO_TYPE)) {
         printf("send STOP gyro to imud failed");
    }
    if (!_imu_client.SendMsgStop(ACCEL_TYPE)) {
         printf("send STOP acc to imud failed");
    }
    _imu_client.~ImuClient();

    printf("sensors client already stop by kill-%d\n", sig);
    exit(0);
}

int main(int argc, char *argv[])
{

    struct imu_pack_dsp imu_data[64];
    int32_t pack_num = 0;
    int rate = 0, freq = 0;

    freq = atof(argv[1]);
    rate = atof(argv[2]);
    printf("set freq %d rate %d\n", freq, rate);

    if (!_imu_client.InitMmap()) {
         printf("init mmap failed");
        return false;
    }

    if (!_imu_client.ConnectServer()) {
         printf("connect imud failed");
        return false;
    }

    if (!_imu_client.SendMsgConfigDataType(ACCEL_TYPE)) {
         printf("send acc CONFIG_TYPE to imud failed");
        return false;
    }

    if (!_imu_client.SendMsgConfigRate(rate)) {
         printf("send acc CONFIG_RATE to imud failed");
        return false;
    }

    if (!_imu_client.SendMsgConfigDataType(ACCEL_TYPE)) {
         printf("send gyro CONFIG_TYPE to imud failed");
        return false;
    }

    if (!_imu_client.SendMsgConfigRate(rate)) {
         printf("send gyro CONFIG_RATE to imud failed");
        return false;
    }

    if (!_imu_client.SendMsgStart(ACCEL_TYPE)) {
         printf("send START accel to imud failed");
        return false;
    }

    if (!_imu_client.SendMsgStart(GYRO_TYPE)) {
         printf("send START gyro to imud failed");
        return false;
    }

    signal(SIGINT, handler);

    while(1)
    {
        _imu_client.GetImuData(imu_data, 64, &pack_num);

        if (pack_num > 0)
        {
            printf("get imu data <%f,%f,%f,%f,%f,%f>, num %d\n",
            imu_data[0].acceloration_x,
            imu_data[0].acceloration_y,
            imu_data[0].acceloration_z,
            imu_data[0].angular_velocity_x,
            imu_data[0].angular_velocity_y,
            imu_data[0].angular_velocity_z,
            pack_num
            );
        }
        usleep(1000*1000/freq);
    }

}