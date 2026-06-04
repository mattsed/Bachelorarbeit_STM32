#ifndef APP_DATA_H
#define APP_DATA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  bool fix_valid;
  int32_t latitude_e7;
  int32_t longitude_e7;
  uint32_t speed_mm_s;
  uint32_t utc_time_ms;
} gnss_data_t;

typedef struct
{
  uint16_t front_raw;
  uint16_t back_raw;
  float front_bar;
  float back_bar;
} brake_pressure_data_t;

typedef struct
{
  int16_t accel_x_raw;
  int16_t accel_y_raw;
  int16_t accel_z_raw;
  int16_t gyro_x_raw;
  int16_t gyro_y_raw;
  int16_t gyro_z_raw;
} imu_data_t;

typedef struct
{
  int16_t accel_x_raw;
  int16_t accel_y_raw;
  int16_t accel_z_raw;
} acc_400g_data_t;

typedef struct
{
  uint32_t timestamp_ms;
  gnss_data_t gnss;
  brake_pressure_data_t brake_pressure;
  imu_data_t imu;
  acc_400g_data_t acc_400g;
} app_sample_t;

#ifdef __cplusplus
}
#endif

#endif /* APP_DATA_H */