#ifndef SEE_see_sensor_H
#define SEE_see_sensor_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct see_sensor_icm4x6xx_imu_data_t {
	float accl_ms2[3];         ///< XYZ acceleration in m/s^2
	float gyro_rad[3];         ///< XYZ gyro rotation in rad/s
    float temperature;         ///< temperature in degrees Celcius
	uint64_t timestamp_slpi_ticks;  ///< timestamp in 19.2 MHz SLPI ticks
	uint64_t timestamp_apps_real;   ///< timestamp in apps proc realtime nanosec
} __attribute__((packed)) see_sensor_icm4x6xx_imu_data_t;

void see_sensor_icm4x6xx_enable_debug_messages();
int  see_sensor_icm4x6xx_detect();
int  see_sensor_icm4x6xx_init();
int  see_sensor_icm4x6xx_read(see_sensor_icm4x6xx_imu_data_t* data);
int  see_sensor_icm4x6xx_close();

typedef struct see_sensor_icp101xx_barometer_data_t {
	float pressure;         ///< pressure in hPa (hectoPascal)
    float temperature;      ///< temperature in degrees Celcius
	uint64_t timestamp_slpi_ticks;  ///< timestamp in 19.2 MHz SLPI ticks
	uint64_t timestamp_apps_real;   ///< timestamp in apps proc realtime nanosec
} __attribute__((packed)) see_sensor_icp101xx_barometer_data_t;

void see_sensor_icp101xx_enable_debug_messages();
int  see_sensor_icp101xx_detect();
int  see_sensor_icp101xx_init();
int  see_sensor_icp101xx_read(see_sensor_icp101xx_barometer_data_t* data);
int  see_sensor_icp101xx_close();

#ifdef __cplusplus
}
#endif

#endif // SEE_see_sensor_H
