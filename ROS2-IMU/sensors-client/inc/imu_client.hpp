/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef _IMU_CLIENT_H_
#define _IMU_CLIENT_H_

#include <stdint.h>
#include <sys/un.h>

#define PACK_GAP 64
#define POLL_NUM_EVENTS 64
#define MMAP_SIZE PACK_GAP * POLL_NUM_EVENTS
#define MMAP_NAME "/run/imu_map"
#define SOCKET_PATH "/run/imud_socket"

struct imu_pack_dsp {
    float acceloration_x;
    float acceloration_y;
    float acceloration_z;
    uint64_t time_acc;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    uint64_t time_gyro;
};

#define IDLE 0
#define INIT 1
#define INIT_RAW 11
#define START 2
#define STOP 3
#define CONFIG_RATE 12
#define CONFIG_DATATYPE 13
#define SET_CONFIG 44

enum {
    ACCEL_TYPE,
    GYRO_TYPE,
    SENSOR_TYPE_MAX,
};

typedef struct {
    uint32_t cmd;
    int32_t data;
} imud_ctrl_msg_t;

class ImuClient
{
public:
    ~ImuClient();
    bool InitMmap();
    bool GetImuData(
        struct imu_pack_dsp* data_array,
        int32_t              max_count,
        int32_t*             returned_sample_count
    );
    bool ConnectServer();
    void DisconnectServer();
    bool SendMsgStart(int sensor);
    bool SendMsgStop(int sensor);
    bool SendMsgConfigRate(int rate);
    bool SendMsgConfigDataType(int type);
    int  ReadMsg(char *buffer, int len);
private:
    bool SendMsg();

    int _mmap_fd;
    char *_map = NULL;
    int _socket_fd;
    imud_ctrl_msg_t _msg = {0, 0};
};

#endif  // _IMU_CLIENT_H_
