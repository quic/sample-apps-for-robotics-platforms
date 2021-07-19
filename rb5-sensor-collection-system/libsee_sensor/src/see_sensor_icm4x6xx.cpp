
#include <iostream>
#include <memory>
#include <atomic>
#include "sns_client.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_cal.pb.h"
#include "sensors_timeutil.h"

#include "see_sensor.h"
#include "see_sensor_icm4x6xx.hpp"

using namespace std;

see_sensor_icm4x6xx icm4x6xx_context;

const sensor_uid see_sensor_icm4x6xx::accel_suid(0x58442ede47acd316, 0x61ab5376b4a5c9aa);
const sensor_uid see_sensor_icm4x6xx::gyro_suid(0x454ade501760a30f, 0x149aec423efc86a1);
const sensor_uid see_sensor_icm4x6xx::temperature_suid(0x59405f698cc47795, 0xb6d1708e9dd07d9f);

void see_sensor_icm4x6xx::process_suid(const string& datatype,
                                const vector<sensor_uid>& suids) {
    int num_suid = suids.size();
    if (debug) cout << "Found " << num_suid << " suids" << endl;

    for (int i = 0; i < num_suid; i++) {
        if (strcmp(datatype.c_str(), ACCEL_SENSOR_TYPE) == 0) {
            if (accel_suid == suids.at(i)) {
                found_suid = true;
                break;
            }
        } else if (strcmp(datatype.c_str(), GYRO_SENSOR_TYPE) == 0) {
            if (gyro_suid == suids.at(i)) {
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

void see_sensor_icm4x6xx::suid_cb(const string& datatype,
                    const vector<sensor_uid>& suids) {
    icm4x6xx_context.process_suid(datatype, suids);
}

void see_sensor_icm4x6xx::error_cb(ssc_error_type error) {
    // TODO: Handle the error condition appropriately
    if (error == SSC_CONNECTION_RESET) {
        cerr << "Connection error: connection reset" << endl;
    } else {
        cerr << "Connection error: unknown error" << endl;
    }
}

void see_sensor_icm4x6xx::resp_cb(uint32_t value) {
    // TODO: Should this be checked for errors?
    // cout << "Got response cb " << value << endl;
}

void see_sensor_icm4x6xx::process_physical_configuration_event(sns_std_sensor_physical_config_event &phy_cfg) {
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

void see_sensor_icm4x6xx::process_sensor_event(sns_std_sensor_event &event, uint64_t ts, sensor_type sensor_event_type) {

    if ( ! configured) {
        if (debug) cout << "Skipping samples. Configuration not complete yet" << endl;
        return;
    }

    // Dump data for debug
    if (debug) {
        if (sensor_event_type == ACCELEROMETER) cout << "ACCL";
        if (sensor_event_type == GYROSCOPE) cout << "GYRO";
        if (sensor_event_type == TEMPERATURE) cout << "TEMP";
        cout << "_DATA,";
        cout << event.status() << ",";
        cout << ts << ",";
        cout << event.data(0);
        if (sensor_event_type == TEMPERATURE) {
            cout << endl;
        } else {
            cout << ",";
            cout << event.data(1) << ",";
            cout << event.data(2) << endl;
        }
    }

    // Don't start recording any data until we see that data from all sensors
    // types is coming in
    if (sensor_event_type == ACCELEROMETER) accel_data_started = true;
    if (sensor_event_type == GYROSCOPE) gyro_data_started = true;
    if (sensor_event_type == TEMPERATURE) temperature_data_started = true;
    if ( ! (accel_data_started && gyro_data_started && temperature_data_started)) return;

    // Once we see that all data streams are coming in wait for
    // valid samples to start recording data in the fifo.
    bool data_valid = (event.status() == SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH);
    if (data_valid) {
        if (sensor_event_type == ACCELEROMETER) accel_data_valid = true;
        if (sensor_event_type == GYROSCOPE) gyro_data_valid = true;
        if (sensor_event_type == TEMPERATURE) {
            temperature_data_valid = true;
            latest_temperature = event.data(0);
            return;
        }
    }

    // Always start recording with accel data.
    if ((sensor_event_type == ACCELEROMETER) && accel_data_valid && \
        gyro_data_valid && temperature_data_valid) start_recording = true;
    if ( ! start_recording) return;

    // Sanity check. This should never happen.
    if ( ! data_valid) {
        cerr << "ERROR: Samples stopped being good!?!" << endl;
        // TODO: Recover from this condition appropriately
        return;
    }

    // Sanity check. This should never happen
    if (event.data_size() != 3) {
        cerr << "ERROR: Invalid amount of data in sensor event" << endl;
        // TODO: Recover from this condition appropriately
        return;
    }

    if (debug) cout << "Data to write" << endl;

    pthread_mutex_lock(&lock);
    if (sensor_event_type == ACCELEROMETER) {
        imu_data[accel_write_index].timestamp_slpi_ticks = ts;
        imu_data[accel_write_index].temperature = latest_temperature;
        imu_data[accel_write_index].accl_ms2[0] = event.data(0);
        imu_data[accel_write_index].accl_ms2[1] = event.data(1);
        imu_data[accel_write_index].accl_ms2[2] = event.data(2);
        increment_index(accel_write_index);

        // If the write index caught up to read index then increment read index.
        if (accel_write_index == imu_data_read_index) increment_index(imu_data_read_index);
        pthread_mutex_unlock(&lock);
    } else { // sensor_event_type == GYROSCOPE
        // Accel data should always come first. Verify a matching timestamp.
        if (imu_data[gyro_write_index].timestamp_slpi_ticks != ts) {
            pthread_mutex_unlock(&lock);
            cerr << "ERROR: gyro sample timing mismatch" << endl;
        } else {
            imu_data[gyro_write_index].gyro_rad[0] = event.data(0);
            imu_data[gyro_write_index].gyro_rad[1] = event.data(1);
            imu_data[gyro_write_index].gyro_rad[2] = event.data(2);
            increment_index(gyro_write_index);
            pthread_mutex_unlock(&lock);
        }
    }
}

void see_sensor_icm4x6xx::process_event(const uint8_t *data, size_t size, uint64_t ts)
{
    sns_client_event_msg pb_event_msg;

    pb_event_msg.ParseFromArray(data, size);

    // Figure out which sensor is sending the event
    const sns_std_suid event_suid = pb_event_msg.suid();
    const sensor_uid event_sensor_suid(event_suid.suid_low(),
                                       event_suid.suid_high());

    sensor_type sensor_event_type = INVALID;
    if (event_sensor_suid == accel_suid) {
        if (debug) cout << "Got accelerometer event" << endl;
        sensor_event_type = ACCELEROMETER;
    } else if (event_sensor_suid == gyro_suid) {
        if (debug) cout << "Got gyroscope event" << endl;
        sensor_event_type = GYROSCOPE;
    } else if (event_sensor_suid == temperature_suid) {
        if (debug) cout << "Got temperature event" << endl;
        sensor_event_type = TEMPERATURE;
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
            process_sensor_event(event, pb_event.timestamp(), sensor_event_type);
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

void see_sensor_icm4x6xx::event_cb(const uint8_t *data, size_t size, uint64_t ts) {
    icm4x6xx_context.process_event(data, size, ts);
}

int see_sensor_icm4x6xx::detect() {

    if (detected) return 0;

    if ( ! detect_suid(suid_cb, ACCEL_SENSOR_TYPE)) {
        cerr << "Couldn't locate SUID for " << ACCEL_SENSOR_TYPE << endl;
        return -1;
    }

    if ( ! detect_suid(suid_cb, GYRO_SENSOR_TYPE)) {
        cerr << "Couldn't locate SUID for " << GYRO_SENSOR_TYPE << endl;
        return -1;
    }

    if ( ! detect_suid(suid_cb, TEMPERATURE_SENSOR_TYPE)) {
        cerr << "Couldn't locate SUID for " << TEMPERATURE_SENSOR_TYPE << endl;
        return -1;
    }

    detected = true;
    return 0;
}

int see_sensor_icm4x6xx::init() {

    if (configured) return 0;

    connection = new ssc_connection(event_cb);
    connection->register_resp_cb(resp_cb);

    pthread_mutex_init(&lock, NULL);

    accel_data_started = false;
    gyro_data_started = false;
    accel_data_valid = false;
    gyro_data_valid = false;
    start_recording = false;

    imu_data_read_index = 0;
    imu_data_read_count = 0;
    accel_write_index = 0;
    gyro_write_index = 0;

    send_config_req(&accel_suid, 500.0, 50000.0);

    int counter = 1000;
    while( ! configuration_succeeded) {
        if ( ! counter--) break;
        usleep(1000);
    }

    if ( ! configuration_succeeded) {
        cerr << "Couldn't configure " << ACCEL_SENSOR_TYPE << " sensor" << endl;
        delete connection;
        return -1;
    }

    configuration_succeeded = false;

    send_config_req(&gyro_suid, 500.0, 50000.0);

    counter = 1000;
    while( ! configuration_succeeded) {
        if ( ! counter--) break;
        usleep(1000);
    }

    if ( ! configuration_succeeded) {
        cerr << "Couldn't configure " << GYRO_SENSOR_TYPE << " sensor" << endl;
        delete connection;
        return -1;
    }

    configuration_succeeded = false;

    send_config_req(&temperature_suid, 5.0, 0.0);

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

int see_sensor_icm4x6xx::read(see_sensor_icm4x6xx_imu_data_t* data) {

    if (data == NULL) return -1;

    // Block until the data starts showing up
    while ( ! start_recording) usleep(1000);

    imu_data_read_count++;

    sensors_timeutil &time_util = sensors_timeutil::get_instance();

    if (debug) {
        if ( ! (imu_data_read_count % 500)) {
            uint64_t apps_ts = time_util.qtimer_ticks_to_elapsedRealtimeNano(imu_data[imu_data_read_index].timestamp_slpi_ticks);
            struct timespec tp;
            clock_gettime(CLOCK_REALTIME, &tp);
            cout << "Sample timestamp:  ";
            cout << imu_data[imu_data_read_index].timestamp_slpi_ticks << endl;
            cout << "Host timestamp:    ";
            cout << apps_ts << endl;
            cout << "Current timestamp: ";
            cout << (tp.tv_sec * 1000000000) + tp.tv_nsec << endl;
        }
    }

    while (imu_data_read_index == gyro_write_index) usleep(100);

    // TODO: Need to figure out how often to do this to deal with drift
    // Shouldn't really ever have to call it because it recalculates on it's own
    // once every minute. This is only needed if it drifts too far in less than
    // a minute.
    // if (time_util.recalculate_offset(true)) {
    //     cout << "Recalculated and offset changed" << endl;
    // }

    if (debug) cout << "Single read at index " << imu_data_read_index << endl;

    uint64_t ticks = imu_data[imu_data_read_index].timestamp_slpi_ticks;
    uint64_t local_time = time_util.qtimer_ticks_to_elapsedRealtimeNano(ticks);
    imu_data[imu_data_read_index].timestamp_apps_real = local_time;

    pthread_mutex_lock(&lock);
    *data = imu_data[imu_data_read_index];
    increment_index(imu_data_read_index);
    pthread_mutex_unlock(&lock);

    return 0;
}

int see_sensor_icm4x6xx::close() {
    if ( ! configured) return 0;
    delete connection;
    configured = false;
    return 0;
}

//------------------------------------------
//      SEE ICM4X6XX API functions
//------------------------------------------

void see_sensor_icm4x6xx_enable_debug_messages() {
    icm4x6xx_context.enable_debug();
}

int see_sensor_icm4x6xx_detect() {
    return icm4x6xx_context.detect();
}

int see_sensor_icm4x6xx_init() {
    return icm4x6xx_context.init();
}

int see_sensor_icm4x6xx_close(){
    return icm4x6xx_context.close();
}

int see_sensor_icm4x6xx_read(see_sensor_icm4x6xx_imu_data_t* data) {
    return icm4x6xx_context.read(data);
}
