/*
 * Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#pragma once
#include <vector>
#include "sensor_connection.h"

struct sensor_uid
{
    sensor_uid() : low(0), high(0) {}
    sensor_uid(uint64_t low, uint64_t high): low(low), high(high) {}
    bool operator==(const sensor_uid& rhs) const
    {
        return (low == rhs.low && high == rhs.high);
    }
    uint64_t low, high;
};

using suid_event_function =
    std::function<void(const std::string& datatype,
                       const std::vector<sensor_uid>& suids)>;

class sns_client
{
public:

    sns_client(suid_event_function cb);
 
    void request_suid(std::string datatype, bool default_only = false);

private:
    suid_event_function _cb;
    void handle_ssc_event(const uint8_t *data, size_t size);
    std::unique_ptr<sensor_connection> _ssc_connect;

    ssc_event_cb_ts get_ssc_event_cb()
    {
        return [this](const uint8_t *data, size_t size, uint64_t ts)
        {
          handle_ssc_event(data, size);
        };
    }
};

