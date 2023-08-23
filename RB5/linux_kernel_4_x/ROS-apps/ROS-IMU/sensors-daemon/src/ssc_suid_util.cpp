/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#include <string>
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "sns_client.pb.h"
#include "ssc_suid_util.h"
#include <cinttypes>
#include <cmath>
#include <chrono>
#include <cutils/properties.h>
#include <fstream>
#include <string>
#include "sns_client.pb.h"
#include "sns_suid.pb.h"
using namespace std;
using namespace google::protobuf::io;

using namespace std::chrono;

suid_lookup::suid_lookup(suid_event_function cb):
    _cb(cb),
    _ssc_conn(get_ssc_event_cb())
{
}

void suid_lookup::request_suid(std::string datatype, bool default_only)
{
    sns_client_request_msg pb_req_msg;
    sns_suid_req pb_suid_req;
    string pb_suid_req_encoded;
    const sensor_uid LOOKUP_SUID = {
        12370169555311111083ull,
        12370169555311111083ull
    };

    sns_logv("requesting suid for %s, ts = %fs", datatype.c_str(),
             duration_cast<duration<float>>(high_resolution_clock::now().
                                            time_since_epoch()).count());

    /* populate SUID request */
    pb_suid_req.set_data_type(datatype);
    pb_suid_req.set_register_updates(true);
    pb_suid_req.set_default_only(default_only);
    pb_suid_req.SerializeToString(&pb_suid_req_encoded);

    /* populate the client request message */
    pb_req_msg.set_msg_id(SNS_SUID_MSGID_SNS_SUID_REQ);
    pb_req_msg.mutable_request()->set_payload(pb_suid_req_encoded);
    pb_req_msg.mutable_suid()->set_suid_high(LOOKUP_SUID.high);
    pb_req_msg.mutable_suid()->set_suid_low(LOOKUP_SUID.low);
    pb_req_msg.mutable_susp_config()->set_delivery_type(SNS_CLIENT_DELIVERY_WAKEUP);
    pb_req_msg.mutable_susp_config()->set_client_proc_type(SNS_STD_CLIENT_PROCESSOR_APSS);
    string pb_req_msg_encoded;
    pb_req_msg.SerializeToString(&pb_req_msg_encoded);
    _ssc_conn.send_request(pb_req_msg_encoded);
}

void suid_lookup::handle_ssc_event(const uint8_t *data, size_t size)
{
    /* parse the pb encoded event */
    sns_client_event_msg pb_event_msg;
    pb_event_msg.ParseFromArray(data, size);
    /* iterate over all events in the message */
    for (int i = 0; i < pb_event_msg.events_size(); i++) {
        auto& pb_event = pb_event_msg.events(i);
        if (pb_event.msg_id() != SNS_SUID_MSGID_SNS_SUID_EVENT) {
            sns_loge("invalid event msg_id=%d", pb_event.msg_id());
            continue;
        }
        sns_suid_event pb_suid_event;
        pb_suid_event.ParseFromString(pb_event.payload());
        const string& datatype =  pb_suid_event.data_type();

        sns_logv("suid_event for %s, num_suids=%d, ts=%fs", datatype.c_str(),
                 pb_suid_event.suid_size(),
                 duration_cast<duration<float>>(high_resolution_clock::now().
                                                time_since_epoch()).count());

        /* create a list of  all suids found for this datatype */
        vector<sensor_uid> suids(pb_suid_event.suid_size());
        for (int j=0; j < pb_suid_event.suid_size(); j++) {
            suids[j] = sensor_uid(pb_suid_event.suid(j).suid_low(),
                                  pb_suid_event.suid(j).suid_high());
        }
        /* send callback for this datatype */
        _cb(datatype, suids);
    }
}
