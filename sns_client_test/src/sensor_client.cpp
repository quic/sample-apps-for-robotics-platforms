/*
 * Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
*/
#include <string>
#include <cinttypes>
#include <cmath>
#include <chrono>
#include <cutils/properties.h>
#include <fstream>
#include "sns_suid.pb.h"
#include "sns_client.pb.h"
#include "sensor_client.h"

using namespace std;
using namespace google::protobuf::io;
using namespace std::chrono;

sns_client::sns_client(suid_event_function cb) : _cb(cb)
{
    _ssc_connect = make_unique<sensor_connection>(get_ssc_event_cb());
}

void sns_client::request_suid(std::string datatype, bool default_only)
{
    sns_client_request_msg req_msg;
    sns_suid_req suid_req;
    string suid_req_encoded;
    const sensor_uid LOOKUP_SUID = {
        12370169555311111083ull,
        12370169555311111083ull
    };

    android_logv("requesting suid for %s, ts = %fs", datatype.c_str(),
             duration_cast<duration<float>>(high_resolution_clock::now().
                                            time_since_epoch()).count());

    /* populate SUID request */
    suid_req.set_data_type(datatype);
    suid_req.set_register_updates(true);
    suid_req.set_default_only(default_only);
    suid_req.SerializeToString(&suid_req_encoded);

    /* populate the client request message */
    req_msg.set_msg_id(SNS_SUID_MSGID_SNS_SUID_REQ);
    req_msg.mutable_request()->set_payload(suid_req_encoded);
    req_msg.mutable_suid()->set_suid_high(LOOKUP_SUID.high);
    req_msg.mutable_suid()->set_suid_low(LOOKUP_SUID.low);
    req_msg.mutable_susp_config()->set_delivery_type(SNS_CLIENT_DELIVERY_WAKEUP);
    req_msg.mutable_susp_config()->set_client_proc_type(SNS_STD_CLIENT_PROCESSOR_APSS);
    string req_msg_encoded;
    req_msg.SerializeToString(&req_msg_encoded);
    _ssc_connect->send_request(req_msg_encoded);
}

void sns_client::handle_ssc_event(const uint8_t *data, size_t size)
{
    /* parse the pb encoded event */
    sns_client_event_msg event_msg;
    event_msg.ParseFromArray(data, size);
    /* iterate over all events in the message */
    for (int i = 0; i < event_msg.events_size(); i++) {
        auto& event = event_msg.events(i);
        if (event.msg_id() != SNS_SUID_MSGID_SNS_SUID_EVENT) {
            android_loge("invalid event msg_id=%d", event.msg_id());
            continue;
        }
        sns_suid_event suid_event;
        suid_event.ParseFromString(event.payload());
        const string& datatype =  suid_event.data_type();

        android_logv("suid_event for %s, num_suids=%d, ts=%fs", datatype.c_str(),
                 suid_event.suid_size(),
                 duration_cast<duration<float>>(high_resolution_clock::now().
                                                time_since_epoch()).count());

        /* create a list of  all suids found for this datatype */
        vector<sensor_uid> suids(suid_event.suid_size());
        for (int j=0; j < suid_event.suid_size(); j++) {
            suids[j] = sensor_uid(suid_event.suid(j).suid_low(),
                                  suid_event.suid(j).suid_high());
        }
        /* send callback for this datatype */
        _cb(datatype, suids);
    }
}
