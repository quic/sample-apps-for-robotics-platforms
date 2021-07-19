
#ifndef SEE_SENSOR_ICP101XX_H
#define SEE_SENSOR_ICP101XX_H

#include "see_sensor_common.hpp"

#define PRESSURE_SENSOR_TYPE "pressure"
#define TEMPERATURE_SENSOR_TYPE "sensor_temperature"

#define DATA_FIFO_SIZE 128

#define tick2us(tick) ((tick)/(192/10))    // 19.2MHz tick frequency

class see_sensor_icp101xx: public see_sensor_common {
public:
    int detect();
    int init();
    int read(see_sensor_icp101xx_barometer_data_t* data);
    int close();

private:
    static void suid_cb(const string& datatype, const vector<sensor_uid>& suids);
    static void resp_cb(uint32_t value);
    static void event_cb(const uint8_t *data, size_t size, uint64_t ts);
    static void error_cb(ssc_error_type error);

    void process_suid(const string& datatype, const vector<sensor_uid>& suids);

    static const sensor_uid pressure_suid;
    static const sensor_uid temperature_suid;

    void process_physical_configuration_event(sns_std_sensor_physical_config_event &phy_cfg);
    void process_sensor_event(sns_std_sensor_event &event, uint64_t ts, bool is_pressure);
    void process_event(const uint8_t *data, size_t size, uint64_t ts);

    bool                    configuration_succeeded;

    bool                    detected;
    bool                    configured;

    pthread_mutex_t         lock;
    see_sensor_icp101xx_barometer_data_t data[DATA_FIFO_SIZE];

    bool                    pressure_data_started;
    bool                    temperature_data_started;
    bool                    pressure_data_valid;
    bool                    temperature_data_valid;
    bool                    start_recording;

    uint32_t                data_read_index;
    uint32_t                data_read_count;
    uint32_t                pressure_write_index;
    uint32_t                temperature_write_index;

    inline void increment_index(uint32_t &index) {
        index++;
        if (index == DATA_FIFO_SIZE) index = 0;
    }
};

#endif // SEE_SENSOR_ICP101XX_H
