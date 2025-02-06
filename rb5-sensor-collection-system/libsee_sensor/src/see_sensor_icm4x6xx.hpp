
#ifndef SEE_SENSOR_ICM4X6XX_H
#define SEE_SENSOR_ICM4X6XX_H

#include "see_sensor_common.hpp"

#define ACCEL_SENSOR_TYPE "accel"
#define GYRO_SENSOR_TYPE "gyro"
#define TEMPERATURE_SENSOR_TYPE "sensor_temperature"

#define IMU_DATA_FIFO_SIZE 1024

typedef enum {
    ACCELEROMETER,
    GYROSCOPE,
    TEMPERATURE,
    INVALID
} sensor_type;

class see_sensor_icm4x6xx: public see_sensor_common {
public:
    int detect();
    int init();
    int read(see_sensor_icm4x6xx_imu_data_t* data);
    int close();

private:

    static void suid_cb(const string& datatype, const vector<sensor_uid>& suids);
    static void resp_cb(uint32_t value);
    static void event_cb(const uint8_t *data, size_t size, uint64_t ts);
    static void error_cb(ssc_error_type error);

    void process_suid(const string& datatype, const vector<sensor_uid>& suids);

    static const sensor_uid accel_suid;
    static const sensor_uid gyro_suid;
    static const sensor_uid temperature_suid;

    void process_physical_configuration_event(sns_std_sensor_physical_config_event &phy_cfg);
    void process_sensor_event(sns_std_sensor_event &event, uint64_t ts, sensor_type sensor_event_type);
    void process_event(const uint8_t *data, size_t size, uint64_t ts);

    bool                    configuration_succeeded;

    bool                    detected;
    bool                    configured;

    pthread_mutex_t         lock;
    see_sensor_icm4x6xx_imu_data_t imu_data[IMU_DATA_FIFO_SIZE];

    float                   latest_temperature;

    bool                    accel_data_started;
    bool                    gyro_data_started;
    bool                    temperature_data_started;
    bool                    accel_data_valid;
    bool                    gyro_data_valid;
    bool                    temperature_data_valid;
    bool                    start_recording;

    uint32_t                imu_data_read_index;
    uint32_t                imu_data_read_count;
    uint32_t                accel_write_index;
    uint32_t                gyro_write_index;

    inline void increment_index(uint32_t &index) {
        index++;
        if (index == IMU_DATA_FIFO_SIZE) index = 0;
    }
};

#endif // SEE_SENSOR_ICM4X6XX_H
