
#include <iostream>
#include <memory>
#include <atomic>
#include "sns_client.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_cal.pb.h"
#include "sensors_timeutil.h"

#include "see_sensor.h"
#include "see_sensor_icp101xx.hpp"

// using namespace google::protobuf::io;
using namespace std;

see_sensor_icp101xx icp101xx_context;

// These SUID can be found using the ssc_sensor_info utility on target.
const sensor_uid see_sensor_icp101xx::pressure_suid(0x72620850aeb26f7e, 0xe9a1184f1593b5ed);
const sensor_uid see_sensor_icp101xx::temperature_suid(0xed40710f53ccc4ea, 0xaa07f41a3f99c392);

void see_sensor_icp101xx::process_suid(const string& datatype,
                                const vector<sensor_uid>& suids) {
    int num_suid = suids.size();
    if (debug) cout << "Found " << num_suid << " suids" << endl;

    for (int i = 0; i < num_suid; i++) {
        if (strcmp(datatype.c_str(), PRESSURE_SENSOR_TYPE) == 0) {
            if (pressure_suid == suids.at(i)) {
                found_suid = true;
                break;
            }
        } else if (strcmp(datatype.c_str(), TEMPERATURE_SENSOR_TYPE) == 0) {
            if (temperature_suid == suids.at(i)) {
                found_suid = true;
                break;
            }
        }

        // if (debug) {
        //     cout << "Received SUID 0x" << hex << suids.at(0).high;
        //     cout << suids.at(0).low << " for ";
        //     cout << datatype.c_str() << endl;
        // }
    }

    if ( ! found_suid) {
        cerr << "Sensor " << datatype.c_str() << " is not available" << endl;
    }
}

void see_sensor_icp101xx::suid_cb(const string& datatype,
                    const vector<sensor_uid>& suids) {
    icp101xx_context.process_suid(datatype, suids);
}

void see_sensor_icp101xx::error_cb(ssc_error_type error) {
    // TODO: Handle the error condition appropriately
    if (error == SSC_CONNECTION_RESET) {
        cerr << "Connection error: connection reset" << endl;
    } else {
        cerr << "Connection error: unknown error" << endl;
    }
}

void see_sensor_icp101xx::resp_cb(uint32_t value) {
    // TODO: Should this be checked for errors?
    // cout << "Got response cb " << value << endl;
}

void see_sensor_icp101xx::process_physical_configuration_event(sns_std_sensor_physical_config_event &phy_cfg) {
    if (debug) {
        cout << "\tRange size " << phy_cfg.range_size() << endl;
        for (int i = 0; i < phy_cfg.range_size(); i++) {
            cout << "\tRange item " << i << " is " << phy_cfg.range(i) << endl;
        }
        if (phy_cfg.has_operation_mode()) {
            cout << "\tOperation mode " << phy_cfg.operation_mode() << endl;
        }
        if (phy_cfg.has_sample_rate()) {
            cout << "\tSample rate " << phy_cfg.sample_rate() << endl;
        }
        if (phy_cfg.has_water_mark()) {
            cout << "\tWatermark " << phy_cfg.water_mark() << endl;
        }
        if (phy_cfg.has_resolution()) {
            cout << "\tResolution " << phy_cfg.resolution() << endl;
        }
        if (phy_cfg.has_active_current()) {
            cout << "\tActive current " << phy_cfg.active_current() << endl;
        }
        if (phy_cfg.has_stream_is_synchronous()) {
            cout << "\tSynchronous stream " << phy_cfg.stream_is_synchronous() << endl;
        }
        if (phy_cfg.has_dri_enabled()) {
            cout << "\tData Ready Interrupt (DRI) enabled " << phy_cfg.dri_enabled() << endl;
        }
        if (phy_cfg.has_dae_watermark()) {
            cout << "\tDAE watermark " << phy_cfg.dae_watermark() << endl;
        }
        if (phy_cfg.has_sync_ts_anchor()) {
            cout << "\tSync TS anchor " << phy_cfg.sync_ts_anchor() << endl;
        }
    }
    configuration_succeeded = true;
}

void see_sensor_icp101xx::process_sensor_event(sns_std_sensor_event &event, uint64_t ts, bool is_pressure) {

    if ( ! configured) {
        if (debug) cout << "Skipping samples. Configuration not complete yet" << endl;
        return;
    }

    // Dump data for debug
    if (debug) {
        if (is_pressure) cout << "PRES";
        else cout << "TEMP";
        cout << "_DATA,";
        cout << event.status() << ",";
        cout << ts << ",";
        cout << event.data(0) << endl;
    }

    // Don't start recording any data until we see that both pressure and temperature
    // samples are coming in
    if (is_pressure) pressure_data_started = true;
    else temperature_data_started = true;
    if ( ! (pressure_data_started && temperature_data_started)) return;

    // Once we see that both data streams are coming in wait for
    // valid samples to start recording data in the fifo.
    bool data_valid = (event.status() == SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH);
    if (data_valid) {
        if (is_pressure) pressure_data_valid = true;
        else temperature_data_valid = true;
    }

    // Always start recording with pressure data.
    if (is_pressure && pressure_data_valid && temperature_data_valid) start_recording = true;
    if ( ! start_recording) return;

    // Sanity check. This should never happen.
    if ( ! data_valid) {
        cerr << "ERROR: Samples stopped being good!?!" << endl;
        // TODO: Recover from this condition appropriately
        return;
    }

    // Sanity check. This should never happen
    if (event.data_size() != 1) {
        cerr << "ERROR: Invalid amount of data in sensor event" << endl;
        // TODO: Recover from this condition appropriately
        return;
    }

    if (debug) cout << "Data to write at index ";

    pthread_mutex_lock(&lock);
    if (is_pressure) {
        data[pressure_write_index].timestamp_slpi_ticks = ts;
        data[pressure_write_index].pressure = event.data(0);

        if (debug) {
            cout << pressure_write_index << ": ";
            cout << data[pressure_write_index].pressure << endl;
        }

        increment_index(pressure_write_index);

        // If the write index caught up to read index then increment read index.
        if (pressure_write_index == data_read_index) increment_index(data_read_index);
        pthread_mutex_unlock(&lock);
    } else {
        // pressure data should always come first. Verify a matching timestamp.
        if (data[temperature_write_index].timestamp_slpi_ticks != ts) {
            pthread_mutex_unlock(&lock);
            cerr << "ERROR: temperature sample timing mismatch" << endl;
        } else {
            data[temperature_write_index].temperature = event.data(0);

            if (debug) {
                cout << temperature_write_index << ": ";
                cout << data[temperature_write_index].temperature << endl;
            }

            increment_index(temperature_write_index);
            pthread_mutex_unlock(&lock);
        }
    }
}

void see_sensor_icp101xx::process_event(const uint8_t *data, size_t size, uint64_t ts)
{
    sns_client_event_msg pb_event_msg;

    pb_event_msg.ParseFromArray(data, size);

    // Figure out which sensor is sending the event
    bool is_pressure = false;
    const sns_std_suid event_suid = pb_event_msg.suid();
    const sensor_uid event_sensor_suid(event_suid.suid_low(),
                                       event_suid.suid_high());
    if (event_sensor_suid == pressure_suid) {
        if (debug) cout << "Got pressure event" << endl;
        is_pressure = true;
    } else if (event_sensor_suid == temperature_suid) {
        if (debug) cout << "Got temperature event" << endl;
    } else {
        cerr << "Got event from unknown source" << endl;
        return;
    }

    // Process all enclosed events
    for(int i = 0; i < pb_event_msg.events_size(); i++)
    {
        const sns_client_event_msg_sns_client_event &pb_event= pb_event_msg.events(i);

        if (SNS_STD_MSGID_SNS_STD_ERROR_EVENT == pb_event.msg_id()) {
            sns_std_error_event error;
            error.ParseFromString(pb_event.payload());
            cerr << "Received standard error event " << error.error() << endl;
        } else if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == \
                   pb_event.msg_id()) {
            sns_std_sensor_event event;
            event.ParseFromString(pb_event.payload());
            process_sensor_event(event, pb_event.timestamp(), is_pressure);
        } else if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT == \
                   pb_event.msg_id()) {
            if (debug) cout << "Received sensor physical configuration event" << endl;
            sns_std_sensor_physical_config_event phy_cfg;
            phy_cfg.ParseFromString(pb_event.payload());
            process_physical_configuration_event(phy_cfg);
        } else if (SNS_CAL_MSGID_SNS_CAL_EVENT == \
                   pb_event.msg_id()) {
            if (debug) cout << "Received cal event" << endl;
        } else if (SNS_STD_MSGID_SNS_STD_FLUSH_EVENT == \
                   pb_event.msg_id()) {
            if (debug) cout << "Received flush event" << endl;
        } else {
          cerr << "Received unknown message ID " << pb_event.msg_id() << endl;
        }
    }
}

void see_sensor_icp101xx::event_cb(const uint8_t *data, size_t size, uint64_t ts) {
    icp101xx_context.process_event(data, size, ts);
}

int see_sensor_icp101xx::detect() {

    if (detected) return 0;

    suid_lookup lookup(suid_cb);
    lookup.request_suid(PRESSURE_SENSOR_TYPE);

    int counter = 10;
    while( ! found_suid) {
        if ( ! counter--) break;
        usleep(100);
    }

    if ( ! found_suid) {
        cerr << "Couldn't locate SUID for " << PRESSURE_SENSOR_TYPE << endl;
        return -1;
    }

    found_suid = false;

    lookup.request_suid(TEMPERATURE_SENSOR_TYPE);

    counter = 10;
    while( ! found_suid) {
        if ( ! counter--) break;
        usleep(100);
    }

    if ( ! found_suid) {
        cerr << "Couldn't locate SUID for " << TEMPERATURE_SENSOR_TYPE << endl;
        return -1;
    }

    detected = true;
    return 0;
}

int see_sensor_icp101xx::init() {

    if (configured) return 0;

    connection = new ssc_connection(event_cb);
    connection->register_resp_cb(resp_cb);

    pthread_mutex_init(&lock, NULL);

    pressure_data_started = false;
    temperature_data_started = false;
    pressure_data_valid = false;
    temperature_data_valid = false;
    start_recording = false;

    data_read_index = 0;
    data_read_count = 0;
    pressure_write_index = 0;
    temperature_write_index = 0;

    send_config_req(&pressure_suid, 25.0, 0);

    int counter = 1000;
    while( ! configuration_succeeded) {
        if ( ! counter--) break;
        usleep(1000);
    }

    if ( ! configuration_succeeded) {
        cerr << "Couldn't configure " << PRESSURE_SENSOR_TYPE << " sensor" << endl;
        delete connection;
        return -1;
    }

    configuration_succeeded = false;

    send_config_req(&temperature_suid, 25.0, 0);

    counter = 1000;
    while( ! configuration_succeeded) {
        if ( ! counter--) break;
        usleep(1000);
    }

    if ( ! configuration_succeeded) {
        cerr << "Couldn't configure " << TEMPERATURE_SENSOR_TYPE << " sensor" << endl;
        delete connection;
        return -1;
    }

    configured = true;
    return 0;
}

int see_sensor_icp101xx::read(see_sensor_icp101xx_barometer_data_t* _data) {

    if (_data == NULL) return -1;

    // Block until the data starts showing up
    while ( ! start_recording) usleep(1000);

    data_read_count++;

    if (debug) {
        if (data_read_index == temperature_write_index) {
            cout << "*** Read pointer caught up to write pointer. ";
            cout << "Waiting for new data. Count = ";
            cout << data_read_count;
            cout << endl;
        }
    }

    sensors_timeutil &time_util = sensors_timeutil::get_instance();
    uint64_t apps_ts = time_util.qtimer_ticks_to_elapsedRealtimeNano(data[data_read_index].timestamp_slpi_ticks);

    while (data_read_index == temperature_write_index) usleep(100);

    if (debug) {
        cout << "Reading at index " << data_read_index;
        cout << ": " << data[data_read_index].timestamp_slpi_ticks;
        cout << " " << data[data_read_index].pressure;
        cout << " " << data[data_read_index].temperature << endl;
    }

    data[data_read_index].timestamp_apps_real = apps_ts;

    pthread_mutex_lock(&lock);
    *_data = data[data_read_index];
    increment_index(data_read_index);
    pthread_mutex_unlock(&lock);

    return 0;
}

int see_sensor_icp101xx::close() {
    if ( ! configured) return 0;
    delete connection;
    configured = false;
    return 0;
}

//------------------------------------------
//      SEE icp10100 API functions
//------------------------------------------

void see_sensor_icp101xx_enable_debug_messages() {
    icp101xx_context.enable_debug();
}

int see_sensor_icp101xx_detect() {
    return icp101xx_context.detect();
}

int see_sensor_icp101xx_init() {
    return icp101xx_context.init();
}

int see_sensor_icp101xx_close(){
    return icp101xx_context.close();
}

int see_sensor_icp101xx_read(see_sensor_icp101xx_barometer_data_t* data) {
    return icp101xx_context.read(data);
}
