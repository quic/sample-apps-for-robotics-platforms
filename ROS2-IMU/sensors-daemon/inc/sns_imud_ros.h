/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef SNS_CLIENT_EXAMPLE_CPP_H
#define SNS_CLIENT_EXAMPLE_CPP_H

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

//commands
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

typedef struct{
	uint32_t cmd;
	int32_t data;
}imud_ctrl_msg_t;

typedef int (*get_raw_data_func_t)(float, float, float);
extern int register_data_callback(uint8_t idx, get_raw_data_func_t data_func);
extern int start_sns_connection(uint8_t idx);
extern int stop_sns_connection(uint8_t idx);
extern int set_sensor_type(uint8_t type);
extern int set_sample_rate(uint16_t rate);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif