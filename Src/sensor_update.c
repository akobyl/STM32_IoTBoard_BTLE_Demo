/*
 * sensor_update.c
 *
 *  Created on: Jun 24, 2019
 *      Author: Andy.Kobyljanec
 */

#include "assert.h"
#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "FreeRTOS_CLI.h"

#include "lsm6dsl_reg.h"
#include "lps22hb_reg.h"
#include "lis3mdl_reg.h"
#include "b_l475e_iot01a_bus.h"

#include "sensor_update.h"

#define SENSOR_UPDATE_PRIORITY    (4)
#define SENSOR_UPDATE_NAME        "sensor update"
#define SENSOR_UPDATE_PERIOD_MS   (50U)

QueueHandle_t sensor_data_queue;

static void sensor_update_task(void * pvParameters);
static BaseType_t sensor_data_console(int8_t *write_buffer,
                                      size_t write_buffer_len, const int8_t *command_str);
static void ftoa(char *output, float n, uint8_t precision_cnt);

static const CLI_Command_Definition_t sensor_data_get_command =
{
    "sensor", "sensor <axis>: Returns data from particular axis\r\n",
    sensor_data_console, 1
};

void sensor_update_init(void)
{
    BaseType_t rtos_success;

    rtos_success = xTaskCreate(sensor_update_task,
                               SENSOR_UPDATE_NAME, 400,
                               NULL,
                               SENSOR_UPDATE_PRIORITY,
                               NULL);

    // Output queue so other tasks can peek at data
    sensor_data_queue = xQueueCreate(1, sizeof(sensorUpdateData_t));

    FreeRTOS_CLIRegisterCommand(&sensor_data_get_command);

    assert(rtos_success == pdPASS);
}

static void sensor_update_task(void * pvParameters)
{
    TickType_t current_time;
    sensorUpdateData_t sensor_data =
    { 0 };

    uint8_t buffer[12];
    BSP_I2C2_Init();

    uint16_t lsm6dsl_address = LSM6DSL_I2C_ADD_L;

    // Query the sensor WHO AM I to verify it is available
    BSP_I2C2_ReadReg(lsm6dsl_address, LSM6DSL_WHO_AM_I, buffer, 1);
    if(buffer[0] != LSM6DSL_ID)
    {
    }

    // Union which allows registers to be cast to integers
    lsm6dsl_reg_t lsm6dsl_reg;

    // Set up the data rate for the accelerometer
    lsm6dsl_reg.byte = 0;
    lsm6dsl_reg.ctrl1_xl.bw0_xl = 0;
    lsm6dsl_reg.ctrl1_xl.lpf1_bw_sel = 0;
    lsm6dsl_reg.ctrl1_xl.fs_xl = 2; // scale of +/- 4g
    lsm6dsl_reg.ctrl1_xl.odr_xl = 1; // 12.5 Hz data rate
    BSP_I2C2_WriteReg(lsm6dsl_address, LSM6DSL_CTRL1_XL, &(lsm6dsl_reg.byte),
                      1);

    // Set up the data rate for the gyroscope
    lsm6dsl_reg.byte = 0;
    lsm6dsl_reg.ctrl2_g.odr_g = 1;  // 12.5 Hz data rate
    lsm6dsl_reg.ctrl2_g.fs_g = 1;   // 500 degree/s full scale
    BSP_I2C2_WriteReg(lsm6dsl_address, LSM6DSL_CTRL2_G, &(lsm6dsl_reg.byte), 1);

    sensor_data.lsm6_initialized = true;

    // Initialize LIS3MDL magnetometer
    uint16_t lis3mdl_address = LIS3MDL_I2C_ADD_H;

    // Query the sensor WHO AM I to verify it is available
    BSP_I2C2_ReadReg(lis3mdl_address, LIS3MDL_WHO_AM_I, buffer, 1);
    if(buffer[0] != LIS3MDL_ID)
    {
        assert(false);
    }

    // Union which allows registers to be cast to integers
    lis3mdl_reg_t lis3mdl_reg;

    lis3mdl_reg.byte = 0;
    lis3mdl_reg.ctrl_reg1.om = 1;       // 500 Hz medium performance mode
    lis3mdl_reg.ctrl_reg1.st = 0;       // no self test
    lis3mdl_reg.ctrl_reg1.temp_en = 1;  // enable the temperature sensor
    BSP_I2C2_WriteReg(lis3mdl_address, LIS3MDL_CTRL_REG1, &(lis3mdl_reg.byte),
                      1);

    lis3mdl_reg.ctrl_reg2.fs = 0;       // use +/- 4 gauss limits
    lis3mdl_reg.ctrl_reg2.reboot = 0;   // do not reboot memory
    lis3mdl_reg.ctrl_reg2.soft_rst = 0; // do not do a reset
    BSP_I2C2_WriteReg(lis3mdl_address, LIS3MDL_CTRL_REG2, &(lis3mdl_reg.byte),
                      1);

    lis3mdl_reg.byte = 0;
    lis3mdl_reg.ctrl_reg3.md =
        1; // single conversion mode (default is powered down 3)
    BSP_I2C2_WriteReg(lis3mdl_address, LIS3MDL_CTRL_REG3, &(lis3mdl_reg.byte),
                      1);

    sensor_data.lis3_initialized = true;

    xQueueOverwrite(sensor_data_queue, &sensor_data);

    for(;;)
    {
        current_time = xTaskGetTickCount();

        // Read all 6 channels from the gyro/accel sensor
        // each channel of data is 2 bytes, so the data is combined
        BSP_I2C2_ReadReg(lsm6dsl_address, LSM6DSL_OUTX_L_G, buffer, 12);
        sensor_data.gyro_x_raw = buffer[0] | (buffer[1] << 8);
        sensor_data.gyro_y_raw = buffer[2] | (buffer[3] << 8);
        sensor_data.gyro_z_raw = buffer[4] | (buffer[5] << 8);
        sensor_data.accel_x_raw = buffer[6] | (buffer[7] << 8);
        sensor_data.accel_y_raw = buffer[8] | (buffer[9] << 8);
        sensor_data.accel_z_raw = buffer[10] | (buffer[11] << 8);

        // Read the magnetometer 3 channels
        BSP_I2C2_ReadReg(lis3mdl_address, LIS3MDL_OUT_X_L, buffer, 8);
        sensor_data.mag_x_raw = buffer[0] | (buffer[1] << 8);
        sensor_data.mag_y_raw = buffer[2] | (buffer[3] << 8);
        sensor_data.mag_z_raw = buffer[4] | (buffer[5] << 8);
        sensor_data.mag_temperature_raw = buffer[6] | (buffer[7] << 8);

        // Convert to engineering units
        sensor_data.gyro_x_mdps = LSM6DSL_FROM_FS_500dps_TO_mdps(
                                      sensor_data.gyro_x_raw);
        sensor_data.gyro_y_mdps = LSM6DSL_FROM_FS_500dps_TO_mdps(
                                      sensor_data.gyro_y_raw);
        sensor_data.gyro_z_mdps = LSM6DSL_FROM_FS_500dps_TO_mdps(
                                      sensor_data.gyro_z_raw);
        sensor_data.accel_x_g = LSM6DSL_FROM_FS_4g_TO_mg(
                                    sensor_data.accel_x_raw);
        sensor_data.accel_y_g = LSM6DSL_FROM_FS_4g_TO_mg(
                                    sensor_data.accel_y_raw);
        sensor_data.accel_z_g = LSM6DSL_FROM_FS_4g_TO_mg(
                                    sensor_data.accel_z_raw);

        sensor_data.mag_x_gauss = lis3mdl_from_fs4_to_gauss(
                                      sensor_data.mag_x_raw);
        sensor_data.mag_y_gauss = lis3mdl_from_fs4_to_gauss(
                                      sensor_data.mag_y_raw);
        sensor_data.mag_z_gauss = lis3mdl_from_fs4_to_gauss(
                                      sensor_data.mag_z_raw);
        sensor_data.mag_temperature_c = lis3mdl_from_lsb_to_celsius(
                                            sensor_data.mag_temperature_c);

        // Send sensor data to queue output
        xQueueOverwrite(sensor_data_queue, &sensor_data);

        vTaskDelayUntil(&current_time, SENSOR_UPDATE_PERIOD_MS);
    }
}

void sensor_data_get(sensorUpdateData_t * sensor_data)
{
    xQueuePeek(sensor_data_queue, sensor_data, 0);
}

static BaseType_t sensor_data_console(int8_t *write_buffer,
                                      size_t write_buffer_len, const int8_t *command_str)
{
    int8_t *pcParameter;
    BaseType_t paramter_str_len;
    sensorUpdateData_t sensor_data;
    float output_value;

    pcParameter = FreeRTOS_CLIGetParameter(command_str, 1, &paramter_str_len);

    uint8_t axis = atoi((const char *) pcParameter);

    sensor_data_get(&sensor_data);

    switch(axis)
    {
    case 0:
        output_value = sensor_data.accel_x_g;
        break;
    case 1:
        output_value = sensor_data.accel_y_g;
        break;
    case 2:
        output_value = sensor_data.accel_z_g;
        break;
    case 3:
        output_value = sensor_data.gyro_x_mdps;
        break;
    case 4:
        output_value = sensor_data.gyro_y_mdps;
        break;
    case 5:
        output_value = sensor_data.gyro_z_mdps;
        break;
    case 6:
        output_value = sensor_data.mag_x_gauss;
        break;
    case 7:
        output_value = sensor_data.mag_y_gauss;
        break;
    case 8:
        output_value = sensor_data.mag_z_gauss;
        break;
    case 9:
        output_value = sensor_data.mag_temperature_c;
        break;
    default:
        break;

    }
    int8_t output_number[20];
    ftoa(output_number, output_value, 3);
    snprintf(write_buffer, write_buffer_len, "%s\r\n", output_number);
    return pdFALSE;
}

static void ftoa(char *output, float n, uint8_t precision_cnt)
{
    char *sign = (n < 0) ? "-" : "";
    n = fabs(n);

    int value = (int) n;
    float fraction = n - value;

    for(int i = 0; i < precision_cnt; i++)
    {
        fraction *= 10;
    }

    if(precision_cnt == 0)
    {
        fraction = 0;
    }

    int fraction_i = (uint16_t) fraction;

    sprintf(output, "%s%d.%d", sign, value, fraction_i);
}
