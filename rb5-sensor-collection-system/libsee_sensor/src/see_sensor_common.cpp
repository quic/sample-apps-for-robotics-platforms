

#include <memory>
#include <iostream>
#include "sns_client.pb.h"
#include "sns_std_sensor.pb.h"
#include "see_sensor_common.hpp"

bool see_sensor_common::detect_suid(suid_event_function suid_cb, string sensor_type) {

    found_suid = false;

    suid_lookup lookup(suid_cb);
    lookup.request_suid(sensor_type);

    int counter = 10;
    while( ! found_suid) {
        if ( ! counter--) break;
        usleep(100);
    }
    return found_suid;
}

void see_sensor_common::send_config_req(sensor_uid const *suid, double sample_rate,
                                        double batch_period) {
    string pb_req_msg_encoded;
    string config_encoded;
    sns_client_request_msg pb_req_msg;
    sns_std_sensor_config config;

    if (debug) cout << "Sending configuration request" << endl;

    config.set_sample_rate(sample_rate);

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
        set_batch_period(batch_period);
    pb_req_msg.SerializeToString(&pb_req_msg_encoded);

    connection->send_request(pb_req_msg_encoded);
}
