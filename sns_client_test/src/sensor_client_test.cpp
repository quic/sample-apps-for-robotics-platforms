/*
 * Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <iostream>
#include <cinttypes>
#include <unistd.h>
#include "sensor_client.h"
#include <string>
#include <unordered_map>
#include <vector>
#include "sns_std_sensor.pb.h"
#include "sns_std_type.pb.h"
#include "sns_client.pb.h"

using namespace std;

#define TEST_TIME 20  // 10 Seconds
#define TEST_SAMPLE_RATE 25  // 10 Hz
#define TEST_BATCH_PERIOD 0  // 0 ms

static int total_samples_rxved = 0;
static sensor_connection *connection;
static const vector<string> sensor_names = {"accel", "gyro"};
static string sensor_name;

/**
 * Event callback function, as registered with sensor_connection.
 */
static void sensor_event_cb(const uint8_t *data, size_t size, uint64_t ts)
{
  sns_client_event_msg pb_event_msg;

  android_loge("Received QMI indication with length %zu", size);

  pb_event_msg.ParseFromArray(data, size);
  for(int i = 0; i < pb_event_msg.events_size(); i++)
  {
    const sns_client_event_msg_sns_client_event &pb_event= pb_event_msg.events(i);
    android_loge("Event[%i] msg_id=%i, ts=%llu", i, pb_event.msg_id(),
        (unsigned long long)pb_event.timestamp());

    if(SNS_STD_MSGID_SNS_STD_ERROR_EVENT == pb_event.msg_id())
    {
      sns_std_error_event error;
      error.ParseFromString(pb_event.payload());

      android_loge("Received error event %i", error.error());
    }
    else if(SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT == pb_event.msg_id())
    {
      sns_std_sensor_physical_config_event config;
      config.ParseFromString(pb_event.payload());

      android_loge("Received config event with sample rate %f", config.sample_rate());
    }
    else if(SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == pb_event.msg_id())
    {
      sns_std_sensor_event event;
      event.ParseFromString(pb_event.payload());
      android_loge("Received sample <%f, %f, %f>",
          event.data(0), event.data(1), event.data(2));
      total_samples_rxved++;
    }
    else
    {
      android_loge("Received unknown message ID %i", pb_event.msg_id());
    }
  }
}

/**
 * Send a SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG to SUID
 */
static void send_config_req(sensor_connection *conn, sensor_uid const *suid)
{
  string req_msg_encoded;
  string config_encoded;
  sns_client_request_msg req_msg;
  sns_std_sensor_config config;

  android_logi("Send config request with sample rate %i and batch period %d",
                TEST_SAMPLE_RATE, TEST_BATCH_PERIOD);

  config.set_sample_rate(TEST_SAMPLE_RATE);
  config.SerializeToString(&config_encoded);

  req_msg.set_msg_id(SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG);
  req_msg.mutable_request()->set_payload(config_encoded);
  req_msg.mutable_suid()->set_suid_high(suid->high);
  req_msg.mutable_suid()->set_suid_low(suid->low);
  req_msg.mutable_susp_config()->set_delivery_type(SNS_CLIENT_DELIVERY_WAKEUP);
  req_msg.mutable_susp_config()->set_client_proc_type(SNS_STD_CLIENT_PROCESSOR_APSS);
  req_msg.mutable_request()->mutable_batching()->set_batch_period(TEST_BATCH_PERIOD);
  req_msg.SerializeToString(&req_msg_encoded);
  conn->send_request(req_msg_encoded);
}

/**
 * see callback as registered with sns_client.
 */
static void see_cb(const std::string& datatype, const std::vector<sensor_uid>& suids)
{
  android_logv("Received SUID event with length %zu", suids.size());
  if(suids.size() > 0)
  {
    sensor_uid suid = suids.at(0);
    connection = new sensor_connection(sensor_event_cb);

    android_logi("Received SUID %" PRIx64 "%" PRIx64 " for '%s'",
        suid.high, suid.low, datatype.c_str());

    printf("Received SUID %" PRIx64 "%" PRIx64 " for '%s' \n",
        suid.high, suid.low, datatype.c_str());
    send_config_req(connection, &suid);
  } else {
    android_logi("%s sensor is not available", sensor_name.c_str());
    cout << "sensor " << sensor_name << " is not available" << endl;
    exit(-1);
  }
}

int main(int argc, char *argv[])
{
  int test_time = TEST_TIME;
  uint32_t sensor_index = 0;
  int opt;
  
  /* parse command line options */
  while ((opt = getopt(argc, argv, "s:t:h")) != -1) {
      switch (opt) {
      case 's':
          sensor_index = atoi(optarg);
          if (sensor_index < 0 || sensor_index >= sensor_names.size()) {
            cout << "Invalid snesor index, sensor index option : [0, " << sensor_names.size() -  1 << "]" << endl;
            return -1;
          }
          break;
      case 't':
          test_time = atoi(optarg);
          break;
      case 'h':
          cout << argv[0] << " option : " << endl;
          cout << "-s [sensor]   sensor = 0 ~ 1 [accel, gyro], default 0" << endl;
          cout << "-t [test_time] senosr test time, default 20 sencods" << endl;
          cout << "-h help" << endl;
          return 0;
      default:
          cout << "Unknown options, default optiins for test" << endl;
          break;
      }
  }

  cout << "<========================= Start sensor client test =========================>\n";

  sensor_name = sensor_names[sensor_index];
  
  cout << "streaming started for " <<  sensor_name << "set SR/RR" << TEST_SAMPLE_RATE << "/" << TEST_BATCH_PERIOD << 
  " and duration " <<  test_time <<  " seconds\n";

  android_logi("streaming started for '%s' set SR/RR '(%d/%d)Hz' and duration '%dSec' ", sensor_name.c_str(),
                            TEST_SAMPLE_RATE, TEST_BATCH_PERIOD, test_time);
  sns_client ssc_client(see_cb);
  ssc_client.request_suid(sensor_name);

  cout << "Wait " << test_time << " seconds to receive sample data in callback" << endl;

  sleep(test_time);
  delete connection;
  android_logi("Received %d samples for '%s' sensor, set SR/RR '(%d/%d)Hz' and duration '%dSec'",
   total_samples_rxved, sensor_name.c_str(), TEST_SAMPLE_RATE, TEST_BATCH_PERIOD, test_time);

  cout << "Received " << total_samples_rxved << " samples for " << sensor_name << "sensor, set SR/RR (" << 
  TEST_SAMPLE_RATE << "/" << TEST_BATCH_PERIOD << ")Hz and duration " << test_time << " seconds \n";
  
  cout << "<========================= Sensor client test done ==========================>\n";

  return 0;
}

 
