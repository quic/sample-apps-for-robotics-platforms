/*
 * Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include "sensor_connection.h"
#include "sensor_client.h"

using namespace std;
using namespace google::protobuf::io;
#define MAX_SVC_INFO_ARRAY_SIZE 5

/* number of times to wait for sensors service */
#define SENSORS_SERVICE_DISCOVERY_TRIES 4
/* timeout for each try */
#define SENSORS_SERVICE_DISCOVERY_TIMEOUT 1

/* exception type defining errors related to qmi API calls */
struct qmi_error : public runtime_error
{
    qmi_error(int error_code, const std::string& what = "") :
        runtime_error(what + ": " + error_code_to_string(error_code)) { }

    static string error_code_to_string(int code)
    {
        static const map<int, string> error_map = {
            { QMI_NO_ERR, "qmi no error" },
            { QMI_INTERNAL_ERR, "qmi internal error" },
            { QMI_TIMEOUT_ERR, "qmi timeout" },
            { QMI_XPORT_BUSY_ERR, "qmi transport busy" },
            { QMI_SERVICE_ERR, "qmi service error" },
        };
        string msg;
        try {
            msg = error_map.at(code);
        } catch (out_of_range& e) {
            msg = "qmi error";
        }
        return msg + " (" + to_string(code) + ")";
    }
};

void see_connection::wait_for_see_service()
{
    qmi_client_type notifier_handle;
    qmi_client_error_type qmi_err;
    qmi_cci_os_signal_type os_params;
    int num_tries = SENSORS_SERVICE_DISCOVERY_TRIES;
    qmi_err = qmi_client_notifier_init(SNS_CLIENT_SVC_get_service_object_v01(),
                                       &os_params, &notifier_handle);
    if (QMI_NO_ERR != qmi_err) {
        throw qmi_error(qmi_err, "qmi_client_notifier_init() failed");
    }
    /* register a callback and wait until service becomes available */
    _service_ready = false;
    qmi_err = qmi_client_register_notify_cb(notifier_handle, see_notify_cb,
                                            this);
    if (qmi_err != QMI_NO_ERR) {
        qmi_client_release(notifier_handle);
        throw qmi_error(qmi_err, "qmi_client_register_notify_cb() failed: %d");
    }
    num_tries = SENSORS_SERVICE_DISCOVERY_TRIES;
    std::unique_lock<std::mutex> lk(_mutex);
    bool timeout = false;
    while (num_tries > 0 && (_service_ready != true)) {
        num_tries--;
        timeout = !_cv.wait_for(lk, std::chrono::seconds(SENSORS_SERVICE_DISCOVERY_TIMEOUT),
                                    [this]{ return _service_ready; });
        if (timeout) {
            if (num_tries == 0) {
                lk.unlock();
                qmi_client_release(notifier_handle);
                throw runtime_error(
                    "FATAL: could not find sensors QMI service");
            }
            android_loge("timeout while waiting for sensors QMI service: "
                     "will try %d more time(s)", num_tries);
        }
    }
    lk.unlock();
    qmi_client_release(notifier_handle);
}

void see_connection::see_connect()
{
    qmi_idl_service_object_type svc_obj =
        SNS_CLIENT_SVC_get_service_object_v01();

    qmi_client_error_type qmi_err;
    qmi_service_info svc_info_array[MAX_SVC_INFO_ARRAY_SIZE];
    uint32_t num_services, num_entries = MAX_SVC_INFO_ARRAY_SIZE;

    /*svc_info_array[5] - Initialized to 0 to avoid static analysis errors*/
    for(uint32_t i = 0 ; i < num_entries ; i++)
      memset(&svc_info_array[i], 0, sizeof(svc_info_array[i]));

    android_logv("waiting for sensors qmi service");
    wait_for_see_service();
    android_logv("connecting to qmi service");
    qmi_err = qmi_client_get_service_list(svc_obj, svc_info_array,
                                          &num_entries, &num_services);
    if (QMI_NO_ERR != qmi_err) {
        throw qmi_error(qmi_err, "qmi_client_get_service_list() failed");
    }

    if (num_entries == 0) {
        throw runtime_error("sensors service has no available instances");
    }

    if (_connection_closed) {
        android_logi("connection got closed do not open qmi_channel");
        return ;
    }

    /* As only one qmi service is expected for sensors, use the 1st instance */
    qmi_service_info svc_info = svc_info_array[0];

    std::unique_lock<std::mutex> lk(_mutex);
    qmi_err = qmi_client_init(&svc_info, svc_obj, see_indication_cb,
                              (void*)this, &_os_params, &_qmi_handle);
    if (qmi_err != QMI_IDL_LIB_NO_ERR) {
        lk.unlock();
        throw qmi_error(qmi_err, "qmi_client_init() failed");
    }

    qmi_err = qmi_client_register_error_cb(_qmi_handle, qmi_error_cb, this);
    if (QMI_NO_ERR != qmi_err) {
        lk.unlock();
        qmi_client_release(_qmi_handle);
        throw qmi_error(qmi_err, "qmi_client_register_error_cb() failed");
    }
    lk.unlock();
    android_logv("connected to ssc for %p", (void *)this);
}

void see_connection::see_disconnect()
{
    std::unique_lock<std::mutex> lk(_mutex);
    if (_qmi_handle != nullptr) {
        qmi_client_release(_qmi_handle);
        _qmi_handle = nullptr;
        /*in ssr call back and sensor disabled , so notify see_connect to comeout*/
        if (_connection_closed)
            _cv.notify_one();
    }
    /*explicit unlock not required , just added not to miss the logic*/
    lk.unlock();
    android_logv("disconnected from ssc for %p", (void *)this);
}

void see_connection::handle_indication(unsigned int msg_id, void *ind_buf,
                                           unsigned int ind_buf_len, uint64_t ts)
{
  int32_t qmi_err;
  size_t ind_size;
  size_t buffer_len;
  uint8_t *buffer = NULL;
  android_loge("msg_id %d " , msg_id);

  if(SNS_CLIENT_REPORT_IND_V01 == msg_id){
    sns_client_report_ind_msg_v01 *ind = NULL;
    ind = (sns_client_report_ind_msg_v01 *)calloc(1, sizeof(sns_client_report_ind_msg_v01));
    if(ind == NULL){
      android_loge("Error while creating memory for ind");
      return;
    }
    ind_size = sizeof(sns_client_report_ind_msg_v01);
    qmi_err = qmi_idl_message_decode(SNS_CLIENT_SVC_get_service_object_v01(),
        QMI_IDL_INDICATION, msg_id, ind_buf,
        ind_buf_len, (void*)ind,
        ind_size);
    if (QMI_IDL_LIB_NO_ERR != qmi_err) {
      android_loge("qmi_idl_message_decode() failed. qmi_err=%d SNS_CLIENT_REPORT_IND_V01", qmi_err);
      free(ind);
      ind = NULL;
      return;
    }
    android_loge("indication, payload_len=%u", ind->payload_len);

    buffer_len = ind->payload_len;
    buffer = (uint8_t*)calloc(1, buffer_len);
    if(buffer == NULL){
      android_loge("buffer failed to creat ");
      free(ind);
      ind = NULL;
      return;
    }
    memcpy(buffer, ind->payload, buffer_len);
    free(ind);
    this->_event_cb_ts(buffer, buffer_len, ts);
    free(buffer);
  }
  else if(SNS_CLIENT_JUMBO_REPORT_IND_V01 == msg_id){
    sns_client_jumbo_report_ind_msg_v01 *ind_jumbo = NULL;
    ind_jumbo = (sns_client_jumbo_report_ind_msg_v01 *)calloc(1, sizeof(sns_client_jumbo_report_ind_msg_v01));
    if(ind_jumbo == NULL){
      android_loge("Error while creating memory for ind_jumbo");
      return;
    }
    ind_size = sizeof(sns_client_jumbo_report_ind_msg_v01);
    qmi_err = qmi_idl_message_decode(SNS_CLIENT_SVC_get_service_object_v01(),
        QMI_IDL_INDICATION, msg_id, ind_buf,
        ind_buf_len, (void*)ind_jumbo,
        ind_size);
    if (QMI_IDL_LIB_NO_ERR != qmi_err) {
      android_loge("qmi_idl_message_decode() failed. qmi_err=%d SNS_CLIENT_JUMBO_REPORT_IND_V01", qmi_err);
      free(ind_jumbo);
      ind_jumbo = NULL;
      return;
    }
    android_loge("indication, payload_len=%u", ind_jumbo->payload_len);
    buffer_len = ind_jumbo->payload_len;
    buffer = (uint8_t*)calloc(1, buffer_len);
    if(buffer == NULL){
      android_loge("buffer failed to creat ind_jumbo ");
      free(ind_jumbo);
      ind_jumbo = NULL;
      return;
    }
    memcpy(buffer, ind_jumbo->payload, buffer_len);
    free(ind_jumbo);
    this->_event_cb_ts(buffer, buffer_len, ts);
    free(buffer);
  }  else{
    android_loge("Unknown indication message ID %i", msg_id);
    return;
  }
}

void see_connection::send_request(const string& pb_req_msg_encoded , bool use_qmi_sync)
{
    sns_client_req_msg_v01 req_msg;
    if (pb_req_msg_encoded.size() > SNS_CLIENT_REQ_LEN_MAX_V01) {
        throw runtime_error("error: payload size too large");
    }

    memcpy(req_msg.payload, pb_req_msg_encoded.c_str(),
           pb_req_msg_encoded.size());

    req_msg.use_jumbo_report_valid = true;
    req_msg.use_jumbo_report = true;
    req_msg.payload_len = pb_req_msg_encoded.size();
    qmi_client_error_type qmi_err;
    sns_client_resp_msg_v01 resp = {};

    if(use_qmi_sync == true) {
      /* send a sync message to ssc */
        qmi_err = qmi_client_send_msg_sync(_qmi_handle, SNS_CLIENT_REQ_V01,
                                         (void*)&req_msg, sizeof(req_msg),
                                         &resp,
                                         sizeof(sns_client_resp_msg_v01),
                                         QMI_RESPONSE_TIMEOUT_MS);
        /*in case of sync - send the response immediately to its clients */
        if(_resp_cb && resp.result_valid)
           _resp_cb(resp.result);
    } else {
      qmi_txn_handle qmi_txn_handle;
      qmi_err = qmi_client_send_msg_async(_qmi_handle, SNS_CLIENT_REQ_V01,
                                       (void*)&req_msg, sizeof(req_msg),
                                       &resp_async,
                                       sizeof(sns_client_resp_msg_v01),
                                       qmi_response_cb,
                                       (void*)this,
                                       &qmi_txn_handle);
    }
    if (qmi_err != QMI_NO_ERR){
        throw qmi_error(qmi_err,
            "qmi_client_send_msg() failed, (client_id=)"
            + to_string(resp.client_id) + ", result="
            + to_string(resp.result));
      }
}

/* creates new connection to ssc */
sensor_connection::sensor_connection(ssc_event_cb_ts event_cb)
{
    _see_conn = new see_connection(event_cb);
    android_logv("ssc connected");
}

sensor_connection::~sensor_connection()
{
    delete _see_conn;
    android_logv("ssc disconnected");
}

/* send encoded client request message to ssc */
void sensor_connection::send_request(const std::string& pb_req_message_encoded , bool use_qmi_sync)
{
  if(_see_conn)
    _see_conn->send_request(pb_req_message_encoded, use_qmi_sync);
  else
    android_loge("_see_conn is NULL");
}

void sensor_connection::register_error_cb(ssc_error_cb cb)
{
    _see_conn->register_error_cb(cb);
}

void sensor_connection::register_resp_cb(ssc_resp_cb cb)
{
    _see_conn->register_resp_cb(cb);
}
