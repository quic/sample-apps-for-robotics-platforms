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
#include <functional>

#include "imu_client.hpp"

#define PACK_SIZE PACK_GAP
static char raw_buf[MMAP_SIZE] = {0};
static uint64_t last_time = 0;

ImuClient::~ImuClient()
{
    munmap(_map, MMAP_SIZE);
    close(_mmap_fd);
    close(_socket_fd);
}

bool ImuClient::InitMmap()
{
    if (_mmap_fd > 0) close(_mmap_fd);
    _mmap_fd = open(MMAP_NAME, O_RDONLY);
    if (_mmap_fd < 0) {
        std::cerr << "imu client: open imu map failed" << std::endl;
        return false;
    }
    _map = (char *)mmap(0, MMAP_SIZE, PROT_READ, MAP_SHARED, _mmap_fd, 0);
    if (_map == NULL) {
        std::cerr << "imu client: mmap failed" << std::endl;
        return false;
    }
    std::cout << "imu client: mmap init succ" << std::endl;
    return true;
}

#define PACK_NUM_MAX (32)
#define DEVIATION_MAX (5000000)
bool ImuClient::GetImuData
(
    struct imu_pack_dsp* dataArray,
    int32_t        max_count,
    int32_t*       returned_sample_count
)
{
    struct imu_pack_dsp *tmp;
    struct imu_pack_dsp *tmpg;
    int i=0;
    int startIndex = -1;
    int availableNum;

    memcpy((void *)raw_buf, (void *)_map, MMAP_SIZE);

    // only get the latest data at beginning
    if(last_time == 0)
    {
        // acc is put on the head of mmap
        tmp = (struct imu_pack_dsp *)(raw_buf + PACK_SIZE*(PACK_NUM_MAX -1));
        // gyro is put on the middle of mmap
        tmpg = (struct imu_pack_dsp *)(raw_buf + PACK_SIZE*(PACK_NUM_MAX*2 -1));
        last_time = tmp->time_acc;

        dataArray[0].acceloration_x = tmp->acceloration_x;
        dataArray[0].acceloration_y = tmp->acceloration_y;
        dataArray[0].acceloration_z = tmp->acceloration_z;
        dataArray[0].angular_velocity_x = tmpg->angular_velocity_x;
        dataArray[0].angular_velocity_y = tmpg->angular_velocity_y;
        dataArray[0].angular_velocity_z = tmpg->angular_velocity_z;
        dataArray[0].time_acc = tmp->time_acc;
        *returned_sample_count = 1;
    }
    else
    {
        *returned_sample_count = 0;

        tmp = (struct imu_pack_dsp *)raw_buf;
        //printf("acc time %ld last %ld\n", tmp->time_acc, last_time);
        if(tmp->time_acc > last_time)
            startIndex = 0;

        if(startIndex < 0)
        {
            i = 0;
            do{
                if(tmp->time_acc == last_time)
                break;

                i++;
                tmp = (struct imu_pack_dsp *)(raw_buf + PACK_SIZE*i);
            }while(i < PACK_NUM_MAX);

            //find index done
            if(i < PACK_NUM_MAX)
            {
                tmp = (struct imu_pack_dsp *)(raw_buf + PACK_SIZE*i);
                i++;
                startIndex = i;
            }
            else
            {
                last_time = 0;
                printf("imu data error\n");
                return false;
            }
        }

        availableNum = PACK_NUM_MAX - startIndex;
        if(availableNum > max_count)
            *returned_sample_count = max_count;
        else
            *returned_sample_count = availableNum;

        for(i = 0; i <*returned_sample_count; i++)
        {
            tmp = (struct imu_pack_dsp *)(raw_buf + PACK_SIZE*(startIndex + i));
            tmpg = (struct imu_pack_dsp *)(raw_buf + PACK_SIZE*(PACK_NUM_MAX + startIndex + i));
            last_time = tmp->time_acc;

            dataArray[i].acceloration_x = tmp->acceloration_x;
            dataArray[i].acceloration_y = tmp->acceloration_y;
            dataArray[i].acceloration_z = tmp->acceloration_z;
            dataArray[i].angular_velocity_x = tmpg->angular_velocity_x;
            dataArray[i].angular_velocity_y = tmpg->angular_velocity_y;
            dataArray[i].angular_velocity_z = tmpg->angular_velocity_z;
            dataArray[i].time_acc = tmp->time_acc;
        }
    }

   return true;
}

bool ImuClient::ConnectServer()
{
    if (_socket_fd > 0) close(_socket_fd);
    _socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (_socket_fd < 0) {
        std::cout << "imu client: create socket failed" << std::endl;
        return false;
    }

    struct sockaddr_un server_addr;
    server_addr.sun_family = AF_UNIX;
    snprintf(server_addr.sun_path, strlen(SOCKET_PATH) + 1, SOCKET_PATH);

    int ret = connect(_socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (ret < 0) {
        std::cout << "imu client: connect socket " << SOCKET_PATH << " failed" << std::endl;
        return false;
    }
    return true;
}

void ImuClient::DisconnectServer()
{
    shutdown(_socket_fd, SHUT_RDWR);
}

bool ImuClient::SendMsg()
{
    int send_len = send(_socket_fd, &_msg, sizeof(_msg), 0);
    if (send_len < 0) {
        std::cout << "imu client: socket send failed" << std::endl;
        return false;
    }
    return true;
}

bool ImuClient::SendMsgStart(int sensor)
{
    _msg.cmd = START;
    _msg.data = sensor;
    return SendMsg();
}

bool ImuClient::SendMsgStop(int sensor)
{
    _msg.cmd = STOP;
    _msg.data = sensor;
    return SendMsg();
}

bool ImuClient::SendMsgConfigRate(int rate)
{
    _msg.cmd = CONFIG_RATE;
    _msg.data = rate;
    return SendMsg();
}

bool ImuClient::SendMsgConfigDataType(int type)
{
    _msg.cmd = CONFIG_DATATYPE;
    _msg.data = type;
    return SendMsg();
}

int ImuClient::ReadMsg(char *buffer, int len)
{
    if (_socket_fd < 0) return -1;
    return read(_socket_fd, buffer, len);
}
