/*=============================================================================
	@file sns_client_example.cpp

	Example client written in C++, using libssc.  Client requests Accel SUID,
	and subsequently sends an enable request to it.  Data will stream for 10
	seconds.

	Copyright (c) 2017-2020 Qualcomm Technologies, Inc.
	All Rights Reserved.
	Confidential and Proprietary - Qualcomm Technologies, Inc.
	===========================================================================*/

/*=============================================================================
	Include Files
	===========================================================================*/
#include <iostream>
#include <cinttypes>
#include <unistd.h>
#include "ssc_suid_util.h"
#include <string>
#include <unordered_map>
#include <vector>
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "sns_std_sensor.pb.h"
#include "sns_std_type.pb.h"
#include "sns_client.pb.h"
#include "sns_imud_ros.h"

using namespace std;
using namespace google::protobuf::io;
/*=============================================================================
	Macro Definitions
	===========================================================================*/

#ifndef UNUSED_VAR
#define UNUSED_VAR(var) ((void)(var));
#endif
enum{
	SENSOR_IDLE,
	SENSOR_RUNNING,
};

#define SENSOR_NAME_LEN (8)
static uint8_t sensor_idx = 0;
#define TEST_LENGTH 20  // 10 Seconds
#define DEFAULT_SAMPLE_RATE 1  // 10 Hz
#define BATCH_PERIOD 0  // 0 ms
static int total_samples_rxved = 0;

typedef struct {
	const char name[SENSOR_NAME_LEN];
	uint8_t state; //0:idle 1:running
	uint16_t sample_rate;
	ssc_connection *connection;
	get_raw_data_func_t callback;
}sensor_info_t;
sensor_info_t sensor_info[SENSOR_TYPE_MAX] = {
	{"accel", SENSOR_IDLE, DEFAULT_SAMPLE_RATE, NULL, NULL},
	{"gyro", SENSOR_IDLE, DEFAULT_SAMPLE_RATE, NULL, NULL},
};
/*=============================================================================
	Static Data
	===========================================================================*/

/*=============================================================================
	Static Function Definitions
	===========================================================================*/

/**
 * Event callback function, as registered with ssc_connection.
 */

static void
event_cb(const uint8_t *data, size_t size, uint64_t ts, get_raw_data_func_t raw_func_callback)
{
	sns_client_event_msg pb_event_msg;
	UNUSED_VAR(ts);

	sns_logv("Received QMI indication with length %zu", size);

	pb_event_msg.ParseFromArray(data, size);
	for(int i = 0; i < pb_event_msg.events_size(); i++)
	{
		const sns_client_event_msg_sns_client_event &pb_event= pb_event_msg.events(i);
		sns_logv("Event[%i] msg_id=%i, ts=%llu", i, pb_event.msg_id(),
				(unsigned long long)pb_event.timestamp());

		if(SNS_STD_MSGID_SNS_STD_ERROR_EVENT == pb_event.msg_id())
		{
			sns_std_error_event error;
			error.ParseFromString(pb_event.payload());

			sns_loge("Received error event %i", error.error());
		}
		else if(SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT == pb_event.msg_id())
		{
			sns_std_sensor_physical_config_event config;
			config.ParseFromString(pb_event.payload());

			sns_loge("Received config event with sample rate %f", config.sample_rate());
		}
		else if(SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == pb_event.msg_id())
		{
			sns_std_sensor_event event;
			event.ParseFromString(pb_event.payload());
			sns_logi("Received sample <%f, %f, %f>",
					event.data(0), event.data(1), event.data(2));
			if (raw_func_callback != NULL)
			{
				raw_func_callback(event.data(0), event.data(1), event.data(2));
			}
			total_samples_rxved++;
		}
		else
		{
			sns_loge("Received unknown message ID %i", pb_event.msg_id());
		}
	}
}

static void
event_cb_accel(const uint8_t *data, size_t size, uint64_t ts)
{
	event_cb(data, size, ts, sensor_info[ACCEL_TYPE].callback);
}

static void
event_cb_gyro(const uint8_t *data, size_t size, uint64_t ts)
{
	event_cb(data, size, ts, sensor_info[GYRO_TYPE].callback);
}

/**
 * Send a SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG to SUID
 */
static void
send_config_req(ssc_connection *conn, sensor_uid const *suid,uint8_t sensor_type)
{
	string pb_req_msg_encoded;
	string config_encoded;
	sns_client_request_msg pb_req_msg;
	sns_std_sensor_config config;

	sns_logi("Send config request with sample rate %i and batch period %d",
		sensor_info[sensor_type].sample_rate, BATCH_PERIOD);

	config.set_sample_rate(sensor_info[sensor_type].sample_rate);
	config.SerializeToString(&config_encoded);

	pb_req_msg.set_msg_id(SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG);
	pb_req_msg.mutable_request()->set_payload(config_encoded);
	pb_req_msg.mutable_suid()->set_suid_high(suid->high);
	pb_req_msg.mutable_suid()->set_suid_low(suid->low);
	pb_req_msg.mutable_susp_config()->
		set_delivery_type(SNS_CLIENT_DELIVERY_WAKEUP);
	pb_req_msg.mutable_susp_config()->
		set_client_proc_type(SNS_STD_CLIENT_PROCESSOR_APSS);
	pb_req_msg.mutable_request()->mutable_batching()->
		set_batch_period(BATCH_PERIOD);
	pb_req_msg.SerializeToString(&pb_req_msg_encoded);
	conn->send_request(pb_req_msg_encoded);
}

/**
 * SUID callback as registered with suid_lookup.
 */
static void
suid_cb(const std::string& datatype, const std::vector<sensor_uid>& suids)
{
	sns_logv("Received SUID event with length %zu", suids.size());
	if(suids.size() > 0)
	{
		sensor_uid suid = suids.at(0);
		if (strcmp(datatype.c_str(), "accel") == 0)
		{
			sensor_info[ACCEL_TYPE].connection = new ssc_connection(event_cb_accel);
			send_config_req(sensor_info[ACCEL_TYPE].connection, &suid, ACCEL_TYPE);
		}
		else
		{
			sensor_info[GYRO_TYPE].connection = new ssc_connection(event_cb_gyro);
			send_config_req(sensor_info[GYRO_TYPE].connection, &suid, GYRO_TYPE);
		}

		sns_logi("Received SUID %" PRIx64 "%" PRIx64 " for '%s'",
				suid.high, suid.low, datatype.c_str());

		printf("Received SUID %" PRIx64 "%" PRIx64 " for '%s' \n",
				suid.high, suid.low, datatype.c_str());
	} else {
		sns_logi("sensor is not available");
		printf("sensor is not available \n");
		exit(-1);
	}
}

int
main(int argc, char *argv[])
{
	return 0;
}

int set_sensor_type(uint8_t type)
{
	if (type >= SENSOR_TYPE_MAX)
	{
		printf("register index is out of range %d\n", type);
		return -1;
	}
	sensor_idx = type;

	return 0;
}

int set_sample_rate(uint16_t rate)
{
	if(rate == 0)
	{
		printf("sample rate is zero\n");
		return -1;
	}

	sensor_info[sensor_idx].sample_rate = rate;

	return 0;
}

int start_sns_connection(uint8_t idx)
{
	if (idx >= SENSOR_TYPE_MAX)
	{
		printf("register index is out of range %d\n", idx);
		return -2;
	}

	if (sensor_info[idx].state == SENSOR_RUNNING)
	{
		printf("repeat start connection\n");
		return -3;
	}

	suid_lookup lookup(suid_cb);
	lookup.request_suid(sensor_info[idx].name);
	sensor_info[idx].state = SENSOR_RUNNING;

	return 0;
}

int stop_sns_connection(uint8_t idx)
{
	if (idx >= SENSOR_TYPE_MAX)
	{
		printf("register index is out of range %d\n", idx);
		return -2;
	}

	if (sensor_info[idx].state == SENSOR_IDLE)
	{
		printf("repeat stop connection\n");
		return -3;
	}
	delete sensor_info[idx].connection;
	sensor_info[idx].state = SENSOR_IDLE;

	return 0;
}

int register_data_callback(uint8_t idx, get_raw_data_func_t data_func)
{
	if (data_func == NULL)
	{
		printf("register func is NULL\n");
		return -1;
	}

	if (idx >= SENSOR_TYPE_MAX)
	{
		printf("register index is out of range %d\n", idx);
		return -2;
	}

	sensor_info[idx].callback = data_func;

	return 0;
}
