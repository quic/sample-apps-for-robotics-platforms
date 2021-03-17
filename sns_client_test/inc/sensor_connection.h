/*
 * Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
*/
#pragma once
#include <functional>
#include <string>
#include <map>
#include <condition_variable>
#include <utils/Log.h>
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "qmi_client.h"
#include "sns_client_api_v01.h"
#include "utils/SystemClock.h"

using namespace std;
using namespace google::protobuf::io;

#define android_loge ALOGE
#define android_logi ALOGI
#define android_logd ALOGD
#define android_logv ALOGV

enum ssc_error_type
{
    SSC_CONNECTION_RESET
};

using ssc_event_cb_ts = std::function<void(const uint8_t *data, size_t size, uint64_t ts)>;

using ssc_error_cb = std::function<void(ssc_error_type error)>;

using ssc_resp_cb = std::function<void(const uint32_t error_value)>;

class see_connection
{
public:
    see_connection(ssc_event_cb_ts event_cb_ts):
        _event_cb_ts(event_cb_ts),
        _connection_closed(false) {
        see_connect();
    }

    ~see_connection(){
        _connection_closed = true;
        see_disconnect();
    }

    void send_request(const string& pb_req_message_encoded, bool use_qmi_sync);
    void register_error_cb(ssc_error_cb cb) { _error_cb = cb; }
    void register_resp_cb(ssc_resp_cb cb) { _resp_cb = cb; }

private:
    ssc_event_cb_ts _event_cb_ts;
    qmi_client_type _qmi_handle = nullptr;
    bool _service_ready;
    std::mutex _mutex;
    std::condition_variable _cv;
    qmi_cci_os_signal_type _os_params;

    bool _connection_closed;
    ssc_error_cb _error_cb;
    ssc_resp_cb _resp_cb;
    static const uint32_t QMI_RESPONSE_TIMEOUT_MS = 2000;
    sns_client_resp_msg_v01 resp_async = {};
    void see_connect();
    void see_disconnect();
    void wait_for_see_service();
    static void see_notify_cb(qmi_client_type user_handle,
                              qmi_idl_service_object_type service_obj,
                              qmi_client_notify_event_type service_event,
                              void *notify_cb_data)
    {
        see_connection *conn = (see_connection *) notify_cb_data;
        unique_lock<mutex> lk(conn->_mutex);
        conn->_service_ready = true;
        conn->_cv.notify_one();
    }

   static void see_indication_cb(qmi_client_type user_handle,
                                              unsigned int msg_id, void* ind_buf,
                                              unsigned int ind_buf_len,
                                              void* ind_cb_data)
   {
       struct timespec ts;
       if (clock_gettime(CLOCK_BOOTTIME, &ts) == -1) {
          perror("clock_gettime");
	        exit(EXIT_FAILURE);
       }

       uint64_t sample_received_ts = 1000000000*(ts.tv_sec) + ts.tv_nsec;
       see_connection *conn = (see_connection*)ind_cb_data;
       conn->handle_indication(msg_id, ind_buf, ind_buf_len, sample_received_ts);
   }

   static void qmi_response_cb(qmi_client_type user_handle,
                                 unsigned int msg_id,
                                 void* resp_cb,
                                 unsigned int resp_cb_len,
                                 void* resp_cb_data,
                                 qmi_client_error_type qmi_err)
   {
     see_connection *conn = (see_connection*)resp_cb_data;
     sns_client_resp_msg_v01 resp = *((sns_client_resp_msg_v01 *)resp_cb);
     if (nullptr != conn){
       if(conn->_resp_cb && resp.result_valid) {
         conn->_resp_cb(resp.result);
       }
     }
   }

   static void qmi_error_cb(qmi_client_type user_handle,
                                         qmi_client_error_type error,
                                         void* err_cb_data)
   {
       see_connection* conn = (see_connection*)err_cb_data;
       android_loge("error=%d", error);
       if (error != QMI_NO_ERR) {
           if (conn->_error_cb)
               conn->_error_cb(SSC_CONNECTION_RESET);
       }
   }

   void handle_indication(unsigned int msg_id,
                           void *ind_buf,
                           unsigned int ind_buf_len,
                           uint64_t ts);
};

class sensor_connection
{
public:

    sensor_connection(ssc_event_cb_ts event_cb);

    ~sensor_connection();

    void send_request(const std::string& pb_req_message_encoded, bool use_qmi_sync = true);

    void register_error_cb(ssc_error_cb cb);

    void register_resp_cb(ssc_resp_cb cb);

private:
    see_connection* _see_conn;

};
