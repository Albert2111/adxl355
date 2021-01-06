/*
 * adxl355_polling_data.c
 *
 *  Created on: Dec 21, 2020
 *      Author: FP
 */
#include <stdio.h>
#include "adxl355.h"
#include "stm32l4xx_hal.h"
#include "spi.h"

#define SENSOR_BUS hspi1

static int32_t platform_read_reg(void* handle, uint8_t reg, uint8_t* pbuf, uint16_t len);
static int32_t platform_write_reg(void* handle, uint8_t reg, uint8_t* pbuf, uint16_t len);

void example_main_adxl355()
{
    dev_ctx_t stm32_dev;
    adxl355_ids_t m_ids;
    stm32_dev.handle = &SENSOR_BUS;
    stm32_dev.read_reg = platform_read_reg;
    stm32_dev.write_reg = platform_write_reg;
    adxl355_soft_reset(&stm32_dev);
    adxl355_ids_get(&stm32_dev, &m_ids);
    printf("ad = 0x%02x mst = 0x%02x partid = 0x%02x revid = 0x%02x\n", m_ids.devid_ad, m_ids.devid_mst, m_ids.partid, m_ids.revid);

    adxl355_mode_set(&stm32_dev, ADXL355_MEASURE);
    HAL_Delay(15);

    float sensor_temp;
    adxl355_temperature_get(&stm32_dev, &sensor_temp);
    printf("temperature = %.2f\n", sensor_temp);

    adxl355_range_t range;
    adxl355_range_get(&stm32_dev, &range);
    printf("range get = %d\n", range);

    adxl355_range_set(&stm32_dev, ADXL355_RANGE_4G);
    printf("set range 4G\n");
    adxl355_range_get(&stm32_dev, &range);
    printf("range get = %d\n", range);

    adxl355_axis_data_t axis_data;
    adxl355_axis_get(&stm32_dev, &axis_data, ADXL355_RANGE_4G);
    printf("x = %d, y = %d, z = %d\n", axis_data.i32_axis_x, axis_data.i32_axis_y, axis_data.i32_axis_z);
    printf("g_x = %f, g_y = %f, g_z = %f\n", axis_data.g_x, axis_data.g_y, axis_data.g_z);

    adxl355_angle_t angle;
    adxl355_angle_get(axis_data, &angle);
    printf("angle x = %f, angle y = %f, angle z = %f\n", angle.angle_x, angle.angle_y, angle.angle_z);

    adxl355_offset_val_t offset;
    adxl355_offset_get(&stm32_dev, &offset, ADXL355_AXIS_X);
    printf("offset get = %d\n", offset.offset_x);

    offset.offset_x = 123;
    adxl355_offset_set(&stm32_dev, offset, ADXL355_AXIS_X);
    printf("offset set = %d\n", offset.offset_x);

    adxl355_offset_get(&stm32_dev, &offset, ADXL355_AXIS_X);
    printf("offset get = %d\n", offset.offset_x);
    adxl355_mode_set(&stm32_dev, ADXL355_STANDBY);
}

static int32_t platform_read_reg(void* handle, uint8_t reg, uint8_t* pbuf, uint16_t len)
{
    uint32_t ret = 0;
    reg = (reg << 1) | ADXL355_READ;
    if(handle == &SENSOR_BUS)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        ret |= HAL_SPI_Transmit (handle, &reg, 1, 1000);
        ret |= HAL_SPI_Receive (handle, pbuf, len, 1000);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }
    else
    {
       ret = 1;
    }
    return ret;
}
static int32_t platform_write_reg(void* handle, uint8_t reg, uint8_t* pbuf, uint16_t len)
{
    uint32_t ret = 0;
    reg = (reg << 1) | ADXL355_WRITE;
    if(handle == &SENSOR_BUS)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        ret |= HAL_SPI_Transmit (handle, &reg, 1, 1000);
        ret |= HAL_SPI_Transmit (handle, pbuf, len, 1000);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }
    else
    {
       ret = 1;
    }
    return ret;
}

int32_t adxl355_fifo_data_get(dev_ctx_t* ctx, adxl355_axis_data_t** fifo_data, uint8_t entries)
{
    uint32_t ret = 0;

    return ret;
}
