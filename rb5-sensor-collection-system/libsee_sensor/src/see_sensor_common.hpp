
#ifndef SEE_SENSOR_COMMON_H
#define SEE_SENSOR_COMMON_H

#include <string>
#include "ssc_connection.h"
#include "ssc_utils.h"

using namespace std;

class see_sensor_common {
public:
    void enable_debug() { debug = true; }

    virtual int detect() = 0;
    virtual int init()   = 0;
    virtual int close()  = 0;

protected:
    bool                    debug;
    bool                    found_suid;
    ssc_connection         *connection;

    void send_config_req(sensor_uid const *suid, double sample_rate_hz,
                         double batch_period);
    bool detect_suid(suid_event_function suid_cb, string sensor_type);
};

#endif // SEE_SENSOR_COMMON_H
