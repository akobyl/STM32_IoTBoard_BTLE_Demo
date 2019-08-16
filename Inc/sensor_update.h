/*
 * sensor_update.h
 *
 *  Created on: Jun 24, 2019
 *      Author: Andy.Kobyljanec
 */

#ifndef SENSOR_UPDATE_H_
#define SENSOR_UPDATE_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t mag_x_raw;
    int16_t mag_y_raw;
    int16_t mag_z_raw;
    int16_t mag_temperature_raw;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_mdps;
    float gyro_y_mdps;
    float gyro_z_mdps;
    float mag_x_gauss;
    float mag_y_gauss;
    float mag_z_gauss;
    float mag_temperature_c;
    bool lsm6_initialized;
    bool lis3_initialized;
} sensorUpdateData_t;

void sensor_update_init(void);

void sensor_data_get(sensorUpdateData_t * sensor_data);

#endif /* SENSOR_UPDATE_H_ */
