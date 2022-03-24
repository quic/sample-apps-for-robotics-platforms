/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#pragma once
#include <vector>
#include <utils/Log.h>

#include "ssc_connection_reference.h"

#define sns_loge ALOGE
#define sns_logi ALOGI
#define sns_logd ALOGD
#define sns_logv ALOGV

#ifndef UNUSED_VAR
#define UNUSED_VAR(var) ((void)(var));
#endif

/**
 * @brief Struct to represent sensor's unique ID (128-bit)
 */
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

/**
 * @brief type alias for an suid event function
 *
 * param datatype: datatype of of the sensor associated with the
 * event
 * param suids: vector of suids available for the given datatype
 */
using suid_event_function =
    std::function<void(const std::string& datatype,
                       const std::vector<sensor_uid>& suids)>;

/**
 * @brief Utility class for discovering available sensors using
 *        dataytpe
 *
 */
class suid_lookup
{
public:
    /**
     * @brief creates a new connection to ssc for suid lookup
     *
     * @param cb callback function for suids
     */
    suid_lookup(suid_event_function cb);

    /**
     *  @brief look up the suid for a given datatype, registered
     *         callback will be called when suid is available for
     *         this datatype
     *
     *  @param datatype data type for which suid is requested
     *  @param default_only option to ask for publishing only default
     *         suid for the given data type. default value is false
     */
    void request_suid(std::string datatype, bool default_only = false);

private:
    suid_event_function _cb;
    void handle_ssc_event(const uint8_t *data, size_t size);
    ssc_connection _ssc_conn;
    ssc_event_cb_ts get_ssc_event_cb()
    {
        return [this](const uint8_t *data, size_t size, uint64_t ts)
        {
          UNUSED_VAR(ts);
          handle_ssc_event(data, size);
        };
    }
};

