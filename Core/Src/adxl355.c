/*
 * adxl355.c
 *
 *  Created on: Dec 21, 2020
 *      Author: 我是一个酸菜鱼
 */
#include "adxl355.h"

/*
 * @brief  读传感器寄存器
 *
 * @param  ctx      设备句柄
 * @param  reg      要读的寄存器
 * @param  data     指向读取缓冲区的指针
 * @param  len      要读的字节数
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_read_reg(dev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
    uint32_t ret = 0;
    ret = ctx->read_reg(ctx->handle, reg, data, len);
    return ret;
}

/*
 * @brief  写传感器寄存器
 *
 * @param  ctx      设备句柄
 * @param  reg      要写的寄存器
 * @param  data     指向要写入数据缓冲区的指针
 * @param  len      要写的字节数
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_write_reg(dev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
    uint32_t ret;
    ret = ctx->write_reg(ctx->handle, reg, data, len);
    return ret;
}

/*
 * @brief  获取传感器各种id
 *
 * @param  ctx      设备句柄
 * @param  ids      用来存储id的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_ids_get(dev_ctx_t* ctx, adxl355_ids_t *ids)
{
    int32_t ret = 0;
    ret |= adxl355_read_reg(ctx, ADXL355_DEVID_AD, &(ids->devid_ad), 1);
    ret |= adxl355_read_reg(ctx, ADXL355_DEVID_MST, &(ids->devid_mst), 1);
    ret |= adxl355_read_reg(ctx, ADXL355_PARTID, &(ids->partid), 1);
    ret |= adxl355_read_reg(ctx, ADXL355_REVID, &(ids->revid), 1);
    return ret;
}

/*
 * @brief  获取传感器各种状态
 *
 * @param  ctx      设备句柄
 * @param  status   用来存储状态值的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_status_get(dev_ctx_t* ctx, adxl355_reg_status_t* status)
{
    int32_t ret = 0;
    ret = adxl355_read_reg(ctx, ADXL355_STATUS, (uint8_t*)status, 1);
    return ret;
}

/*
 * @brief  获取fifo内已经存在的条目数
 *
 * @param  ctx      设备句柄
 * @param  status   用来存储条目数的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_fifo_entries_get(dev_ctx_t* ctx, uint8_t* entries)
{
    int32_t ret = 0;
    ret = adxl355_read_reg(ctx, ADXL355_FIFO_ENTRIES, entries, 1);
    return ret;
}

/*
 * @brief  获取传感器内部工作温度
 *
 * @param  ctx      设备句柄
 * @param  status   用来存储温度的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_temperature_get(dev_ctx_t* ctx, float* temperature)
{
    int32_t ret = 0;
    uint8_t temp1, temp2;
    uint16_t temp_adc;
    ret |= adxl355_read_reg(ctx, ADXL355_TEMP2, &temp2, 1);
    ret |= adxl355_read_reg(ctx, ADXL355_TEMP1, &temp1, 1);
    temp_adc = (((uint16_t)temp2) << 8) | temp1;
    *temperature = ((((float)temp_adc - ADXL355_TEMP_BIAS)) / ADXL355_TEMP_SLOPE) + 25.0;
    return ret;
}

/*
 * @brief  获取传感器工作状态
 *
 * @param  ctx      设备句柄
 * @param  status   用来存储传感器工作状态的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_mode_get(dev_ctx_t* ctx, adxl355_mode_status* mode)
{
    int32_t ret = 0;
    adxl355_reg_power_t ctl;
    ret = adxl355_read_reg(ctx, ADXL355_POWER_CTL, (uint8_t*)&ctl, 1);
    if(ret == 0)
    {
        *mode = ctl.standby;
    }
    return ret;
}

/*
 * @brief  设置传感器工作状态
 *
 * @param  ctx      设备句柄
 * @param  status   状态值
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_mode_set(dev_ctx_t* ctx, adxl355_mode_status mode)
{
    int32_t ret = 0;
    adxl355_reg_power_t ctl;
    ret = adxl355_read_reg(ctx, ADXL355_POWER_CTL, (uint8_t*)&ctl, 1);
    if(ret == 0)
    {
        ctl.standby = mode;
        ret |= adxl355_write_reg(ctx, ADXL355_POWER_CTL, (uint8_t*)&ctl, 1);
    }
    return ret;
}

/*
 * @brief  传感器软复位
 *
 * @param  ctx      设备句柄
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_soft_reset(dev_ctx_t* ctx)
{
    int32_t ret = 0;
    uint8_t rst;
    rst = PD_RST;
    ret = adxl355_write_reg(ctx, ADXL355_ADX_RESET, &rst, 1);
    return ret;
}

/*
 * @brief  温度处理单元开关状态获取
 *
 * @param  ctx      设备句柄
 * @param  status   用来存储传温度处理单元开关状态的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_temp_off_get(dev_ctx_t* ctx, adxl355_temp_status* status)
{
    int32_t ret = 0;
    adxl355_reg_power_t ctl;
    ret = adxl355_read_reg(ctx, ADXL355_POWER_CTL, (uint8_t*)&ctl, 1);
    if(ret == 0)
    {
        *status = ctl.temp_off;
    }
    return ret;
}

/*
 * @brief  设置温度处理单元开关状态
 *
 * @param  ctx      设备句柄
 * @param  status   温度处理单元开关状态值
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_temp_off_set(dev_ctx_t* ctx, adxl355_temp_status status)
{
    int32_t ret = 0;
    adxl355_reg_power_t ctl;
    ret = adxl355_read_reg(ctx, ADXL355_POWER_CTL, (uint8_t*)&ctl, 1);
    if(ret == 0)
    {
        ctl.temp_off = status;
        ret |= adxl355_write_reg(ctx, ADXL355_POWER_CTL, (uint8_t*)&ctl, 1);
    }
    return ret;
}

/*
 * @brief  强制DRDY输出0 状态获取
 *
 * @param  ctx      设备句柄
 * @param  status   用来存储状态的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_drdy_off_get(dev_ctx_t* ctx, adxl355_drdy_status* status)
{
    int32_t ret = 0;
    adxl355_reg_power_t ctl;
    ret = adxl355_read_reg(ctx, ADXL355_POWER_CTL, (uint8_t*)&ctl, 1);
    if(ret == 0)
    {
        *status = ctl.drdy_off;
    }
    return ret;
}

/*
 * @brief  设置强制DRDY输出0状态
 *
 * @param  ctx      设备句柄
 * @param  status   状态值
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_drdy_off_set(dev_ctx_t* ctx, adxl355_drdy_status status)
{
    int32_t ret = 0;
    adxl355_reg_power_t ctl;
    ret = adxl355_read_reg(ctx, ADXL355_POWER_CTL, (uint8_t*)&ctl, 1);
    if(ret == 0)
    {
        ctl.drdy_off = status;
        ret |= adxl355_write_reg(ctx, ADXL355_POWER_CTL, (uint8_t*)&ctl, 1);
    }
    return ret;
}

/*
 * @brief  将传感器内部读出的无符号数转换成有符号数
 *
 * @param  axis_data      传入的无符号数
 * @retval                转换后的有符号值
 *
 */
int32_t adxl355_acceleration_data_conversion (uint32_t axis_data)
{
    int32_t volatile i32conversion = 0;
    axis_data = ( axis_data >> 4);
    axis_data = (axis_data & 0x000FFFFF);
   if((axis_data & 0x00080000) == 0x00080000)         //checking if most sig bit is set
   {
        i32conversion = (axis_data | 0xFFF00000);     //if its set, we try to make it negative
   }
   else
   {
        i32conversion = axis_data;
   }
   return i32conversion;
}

/*
 * @brief  加速度值转换成角度值
 *
 * @param  axis_data      传入的角度数据
 * @param  angles         用来存储角度值的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_angle_get(adxl355_axis_data_t axis_data, adxl355_angle_t* angles)
{
    int32_t ret = 0;

    angles->angle_x = atan(axis_data.i32_axis_x / (sqrt(pow(axis_data.i32_axis_y, 2) + pow(axis_data.i32_axis_z, 2))));
    angles->angle_x *= (180.0 / M_PI);

    angles->angle_y = atan(axis_data.i32_axis_y / (sqrt(pow(axis_data.i32_axis_x, 2) + pow(axis_data.i32_axis_z,2))));
    angles->angle_y *= (180.0 / M_PI);

    angles->angle_z = atan((sqrt(pow(axis_data.i32_axis_x, 2) + pow(axis_data.i32_axis_y, 2))) / axis_data.i32_axis_z);
    angles->angle_z *= (180.0 / M_PI);

    return ret;
}

/*
 * @brief  将传感器数据转换成重力加速度G
 *
 * @param  i32_axis_val   传入的有符号AD值
 * @retval                转换后的加速度值
 *
 */
float adxl355_fs2_to_g(int32_t i32_axis_val)
{
    return i32_axis_val / ADXL355_SCALE_2G;
}

float adxl355_fs4_to_g(int32_t i32_axis_val)
{
    return i32_axis_val / ADXL355_SCALE_4G;
}

float adxl355_fs8_to_g(int32_t i32_axis_val)
{
    return i32_axis_val / ADXL355_SCALE_8G;
}

/*
 * @brief  获取各轴的值（包括原始值，有符号值，和加速度值）
 *
 * @param  ctx          设备句柄
 * @param  axis_data    用来存储各轴值的指针
 * @param  range        当前量程范围
 * @retval              返回值(返回0->没有错误).
 *
 */
int32_t adxl355_axis_get(dev_ctx_t* ctx, adxl355_axis_data_t* axis_data, adxl355_range_t range)
{
    int32_t ret = 0;
    uint8_t rx_buff[3] = {0};
    uint32_t ui32value_l = 0;
    uint32_t ui32value_m = 0;
    uint32_t ui32value_h = 0;

    ret = adxl355_read_reg(ctx, ADXL355_XDATA3, rx_buff, 3);
    if(ret == 0)
    {
        ui32value_l = rx_buff[2];
        ui32value_m = rx_buff[1];
        ui32value_h = rx_buff[0];
        axis_data->raw_axis_x = (ui32value_h << 16) + (ui32value_m << 8) + ui32value_l;
        axis_data->i32_axis_x = adxl355_acceleration_data_conversion(axis_data->raw_axis_x);
    }

    ret |= adxl355_read_reg(ctx, ADXL355_YDATA3, rx_buff, 3);
    if(ret == 0)
    {
        ui32value_l = rx_buff[2];
        ui32value_m = rx_buff[1];
        ui32value_h = rx_buff[0];
        axis_data->raw_axis_y = (ui32value_h << 16) + (ui32value_m << 8) + ui32value_l;
        axis_data->i32_axis_y = adxl355_acceleration_data_conversion(axis_data->raw_axis_y);
    }

    ret |= adxl355_read_reg(ctx, ADXL355_ZDATA3, rx_buff, 3);
    if(ret == 0)
    {
        ui32value_l = rx_buff[2];
        ui32value_m = rx_buff[1];
        ui32value_h = rx_buff[0];
        axis_data->raw_axis_z = (ui32value_h << 16) + (ui32value_m << 8) + ui32value_l;
        axis_data->i32_axis_z = adxl355_acceleration_data_conversion(axis_data->raw_axis_z);
        if(range == ADXL355_RANGE_2G)
        {
            axis_data->g_x = adxl355_fs2_to_g(axis_data->i32_axis_x);
            axis_data->g_y = adxl355_fs2_to_g(axis_data->i32_axis_y);
            axis_data->g_z = adxl355_fs2_to_g(axis_data->i32_axis_z);
        }
        if(range == ADXL355_RANGE_4G)
        {
            axis_data->g_x = adxl355_fs4_to_g(axis_data->i32_axis_x);
            axis_data->g_y = adxl355_fs4_to_g(axis_data->i32_axis_y);
            axis_data->g_z = adxl355_fs4_to_g(axis_data->i32_axis_z);
        }
        if(range == ADXL355_RANGE_8G)
        {
            axis_data->g_x = adxl355_fs8_to_g(axis_data->i32_axis_x);
            axis_data->g_y = adxl355_fs8_to_g(axis_data->i32_axis_y);
            axis_data->g_z = adxl355_fs8_to_g(axis_data->i32_axis_z);
        }
    }
    else
    {
        axis_data->g_x = 0xFFFFFFFF;
        axis_data->g_y = 0xFFFFFFFF;
        axis_data->g_z = 0xFFFFFFFF;
        axis_data->i32_axis_x = 0xFFFFFFFF;
        axis_data->i32_axis_y = 0xFFFFFFFF;
        axis_data->i32_axis_z = 0xFFFFFFFF;
        axis_data->raw_axis_x = 0xFFFFFFFF;
        axis_data->raw_axis_y = 0xFFFFFFFF;
        axis_data->raw_axis_z = 0xFFFFFFFF;
    }
    return ret;
}

/*
 * @brief  获取传感器当前量程范围
 *
 * @param  ctx      设备句柄
 * @param  range    用来存储范围值的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_range_get(dev_ctx_t* ctx, adxl355_range_t* range)
{
    int32_t ret = 0;
    adxl355_reg_range_t range_mix_reg;
    ret = adxl355_read_reg(ctx, ADXL355_RANGE, (uint8_t*)&range_mix_reg, 1);
    if(ret == 0)
    {
        switch (range_mix_reg.range)
        {
            case ADXL355_RANGE_2G:
                *range = ADXL355_RANGE_2G;
                break;
            case ADXL355_RANGE_4G:
                *range = ADXL355_RANGE_4G;
                break;
            case ADXL355_RANGE_8G:
                *range = ADXL355_RANGE_8G;
                break;
            default:
                ret = -1;
                break;
        }
    }
    return ret;
}

/*
 * @brief  设置传感器量程范围
 *
 * @param  ctx      设备句柄
 * @param  range    量程值
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_range_set(dev_ctx_t* ctx, adxl355_range_t range)
{
    int32_t ret = 0;
    adxl355_reg_range_t range_mix_reg;
    ret = adxl355_read_reg(ctx, ADXL355_RANGE, (uint8_t*)&range_mix_reg, 1);
    if(ret == 0)
    {
        range_mix_reg.range = range;
        ret |= adxl355_write_reg(ctx, ADXL355_RANGE, (uint8_t*)&range_mix_reg, 1);
    }
    return ret;
}

/*
 * @brief  获取传感器当前滤波器和数据输出速率
 *
 * @param  ctx      设备句柄
 * @param  hpf_odr  用来存储滤波器和数据输出速率值的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_hpf_odr_get(dev_ctx_t* ctx, adxl355_hpf_odr_t* hpf_odr)
{
    int32_t ret = 0;
    adxl355_reg_filter_t filter_reg;
    ret = adxl355_read_reg(ctx, ADXL355_FILTER, (uint8_t*)&filter_reg, 1);
    if(ret == 0)
    {
        hpf_odr->hpf = filter_reg.hpf_corner;
        hpf_odr->odr_lpf = filter_reg.odr_lpf;
    }
    return ret;
}

/*
 * @brief  设置传感器当前滤波器和数据输出速率
 *
 * @param  ctx      设备句柄
 * @param  hpf_odr  滤波器和数据输出速率的值
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_hpf_odr_set(dev_ctx_t* ctx, adxl355_hpf_odr_t hpf_odr)
{
    int32_t ret = 0;
    adxl355_reg_filter_t filter_reg;
    ret = adxl355_read_reg(ctx, ADXL355_FILTER, (uint8_t*)&filter_reg, 1);
    if(ret == 0)
    {
        filter_reg.hpf_corner = hpf_odr.hpf;
        filter_reg.odr_lpf = hpf_odr.odr_lpf;
        ret |= adxl355_write_reg(ctx, ADXL355_FILTER, (uint8_t*)&filter_reg, 1);
    }
    return ret;
}

/*
 * @brief  获取传感器中断表
 *
 * @param  ctx      设备句柄
 * @param  int_map  存储中断表的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_intmap_get(dev_ctx_t* ctx, adxl355_reg_intmap_t* int_map)
{
    int32_t ret = 0;
    ret = adxl355_read_reg(ctx, ADXL355_INT_MAP, (uint8_t*)int_map, 1);
    return ret;
}

/*
 * @brief  设置传感器中断表
 *
 * @param  ctx      设备句柄
 * @param  int_map  中断表的值
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_intmap_set(dev_ctx_t* ctx, adxl355_reg_intmap_t int_map)
{
    int32_t ret = 0;
    ret = adxl355_write_reg(ctx, ADXL355_INT_MAP, (uint8_t*)&int_map, 1);
    return ret;
}

/*
 * @brief  获取外部时钟启用状态
 *
 * @param  ctx      设备句柄
 * @param  state    指向用于存储状态的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_exclk_get(dev_ctx_t* ctx, adxl355_state_t* state)
{
    int32_t ret = 0;
    adxl355_reg_sync_t sync_reg;
    ret = adxl355_read_reg(ctx, ADXL355_SYNC, (uint8_t*)&sync_reg, 1);
    if(ret == 0)
    {
        *state = sync_reg.ext_clk;
    }
    return ret;
}

/*
 * @brief  设置外部时钟启用状态
 *
 * @param  ctx      设备句柄
 * @param  state    状态值
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_exclk_set(dev_ctx_t* ctx, adxl355_state_t state)
{
    int32_t ret = 0;
    adxl355_reg_sync_t sync_reg;
    ret = adxl355_read_reg(ctx, ADXL355_SYNC, (uint8_t*)&sync_reg, 1);
    if(ret == 0)
    {
        sync_reg.ext_clk = state;
        ret |= adxl355_write_reg(ctx, ADXL355_SYNC, (uint8_t*)&sync_reg, 1);
    }
    return ret;
}

/*
 * @brief  数据同步类型获取
 *
 * @param  ctx      设备句柄
 * @param  sync     指向存储同步类型的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_sync_get(dev_ctx_t* ctx, adxl355_sync_t* sync)
{
    int32_t ret = 0;
    adxl355_reg_sync_t sync_reg;
    ret = adxl355_read_reg(ctx, ADXL355_SYNC, (uint8_t*)&sync_reg, 1);
    if(ret == 0)
    {
        *sync = sync_reg.ext_sync;
    }
    return ret;
}

/*
 * @brief  设置数据同步类型
 *
 * @param  ctx      设备句柄
 * @param  sync     同步类型值
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_sync_set(dev_ctx_t* ctx, adxl355_sync_t sync)
{
    int32_t ret = 0;
    adxl355_reg_sync_t sync_reg;
    ret = adxl355_read_reg(ctx, ADXL355_SYNC, (uint8_t*)&sync_reg, 1);
    if(ret == 0)
    {
        sync_reg.ext_sync = sync;
        ret |= adxl355_write_reg(ctx, ADXL355_SYNC, (uint8_t*)&sync_reg, 1);
    }
    return ret;
}

/*
 * @brief  获取传感器自检状态
 *
 * @param  ctx      设备句柄
 * @param  st_state 指向存储自检状态的指针
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_st_get(dev_ctx_t* ctx, adxl355_st_state_t* st_state)
{
    int32_t ret = 0;
    adxl355_reg_st_t st_reg;
    ret = adxl355_read_reg(ctx, ADXL355_SELF_TEST, (uint8_t*)&st_reg, 1);
    if(ret == 0)
    {
        st_state->st1 = st_reg.st1;
        st_state->st2 = st_reg.st2;
    }
    return ret;
}

/*
 * @brief  设置传感器自检状态
 *
 * @param  ctx      设备句柄
 * @param  st_state 自检状态值
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_st_set(dev_ctx_t* ctx,adxl355_st_state_t st_state)
{
    int32_t ret = 0;
    adxl355_reg_st_t st_reg;
    ret = adxl355_read_reg(ctx, ADXL355_SELF_TEST, (uint8_t*)&st_reg, 1);
    if(ret == 0)
    {
        st_reg.st1 = st_state.st1;
        st_reg.st2 = st_state.st2;
        ret |= adxl355_write_reg(ctx, ADXL355_SELF_TEST, (uint8_t*)&st_reg, 1);
    }
    return ret;
}

/*
 * @brief  获取传感器offset值
 *
 * @param  ctx      设备句柄
 * @param  offset   指向存储offset值的指针
 * @param  axis     要获取offset值的轴
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_offset_get(dev_ctx_t* ctx, adxl355_offset_val_t* offset, adxl355_axis_t axis)
{
    int32_t ret = 0;
    uint8_t tmp[2] = {0};
    switch(axis)
    {
        case ADXL355_AXIS_X:
            ret = adxl355_read_reg(ctx, ADXL355_OFFSET_X_L, tmp, 1);
            ret |= adxl355_read_reg(ctx, ADXL355_OFFSET_X_H, tmp+1 , 1);
            offset->offset_x = (((int16_t)tmp[1]) << 8) | (int16_t)tmp[0];
            break;
        case ADXL355_AXIS_Y:
            ret = adxl355_read_reg(ctx, ADXL355_OFFSET_Y_L, tmp, 1);
            ret |= adxl355_read_reg(ctx, ADXL355_OFFSET_Y_H, tmp+1 , 1);
            offset->offset_y = (((int16_t)tmp[1]) << 8) | (int16_t)tmp[0];
            break;
        case ADXL355_AXIS_Z:
            ret = adxl355_read_reg(ctx, ADXL355_OFFSET_Z_L, tmp, 1);
            ret |= adxl355_read_reg(ctx, ADXL355_OFFSET_Z_H, tmp+1 , 1);
            offset->offset_z = (((int16_t)tmp[1]) << 8) | (int16_t)tmp[0];
            break;
        default:
            ret = -1;
            break;
    }
    return ret;
}

/*
 * @brief  设置传感器offset值
 *
 * @param  ctx      设备句柄
 * @param  offset   offset值
 * @param  axis     要设置offset值的轴
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_offset_set(dev_ctx_t* ctx, adxl355_offset_val_t offset, adxl355_axis_t axis)
{
    int32_t ret = 0;
    uint8_t tmp[2] = {0};
    switch (axis)
    {
        case ADXL355_AXIS_X:
            tmp[0] = (offset.offset_x) & 0xFF;
            ret = adxl355_write_reg(ctx, ADXL355_OFFSET_X_L, tmp, 1);
            tmp[1] = (offset.offset_x) >> 8;
            ret |= adxl355_write_reg(ctx, ADXL355_OFFSET_X_H, tmp+1, 1);
            break;
        case ADXL355_AXIS_Y:
            tmp[0] = (offset.offset_y) & 0xFF;
            ret = adxl355_write_reg(ctx, ADXL355_OFFSET_Y_L, tmp, 1);
            tmp[1] = (offset.offset_y) >> 8;
            ret |= adxl355_write_reg(ctx, ADXL355_OFFSET_Y_H, tmp+1, 1);
            break;
        case ADXL355_AXIS_Z:
            tmp[0] = (offset.offset_z) & 0xFF;
            ret = adxl355_write_reg(ctx, ADXL355_OFFSET_Z_L, tmp, 1);
            tmp[1] = (offset.offset_z) >> 8;
            ret |= adxl355_write_reg(ctx, ADXL355_OFFSET_Z_H, tmp+1, 1);
            break;
        default:
            ret = -1;
            break;
    }
    return ret;
}

/*
 * @brief  活动检测算法开关状态获取
 *
 * @param  ctx      设备句柄
 * @param  state    指向存储活动算法状态的指针
 * @param  axis     要获取活动算法是否激活的轴
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_act_get(dev_ctx_t* ctx, adxl355_state_t* state, adxl355_axis_t axis)
{
    int32_t ret = 0;
    adxl355_reg_act_t act_reg;
    ret = adxl355_read_reg(ctx, ADXL355_ACT_EN, (uint8_t*)&act_reg, 1);
    switch (axis)
    {
        case ADXL355_AXIS_X:
            *state = act_reg.act_x;
            break;
        case ADXL355_AXIS_Y:
            *state = act_reg.act_y;
            break;
        case ADXL355_AXIS_Z:
            *state = act_reg.act_z;
            break;
        default:
            ret = -1;
            break;
    }
    return ret;
}

/*
 * @brief  设置活动检测算法开关状态
 *
 * @param  ctx      设备句柄
 * @param  state    状态值
 * @param  axis     要激活活动算法的轴
 * @retval ret      返回值(返回0->没有错误).
 *
 */
int32_t adxl355_act_set(dev_ctx_t* ctx, adxl355_state_t state, adxl355_axis_t axis)
{
    int32_t ret = 0;
    adxl355_reg_act_t act_reg;
    ret = adxl355_read_reg(ctx, ADXL355_ACT_EN, (uint8_t*)&act_reg, 1);
    if(ret == 0)
    {
        switch (axis)
        {
            case ADXL355_AXIS_X:
                act_reg.act_x = state;
                ret |= adxl355_write_reg(ctx, ADXL355_ACT_EN, (uint8_t*)&act_reg, 1);
                break;
            case ADXL355_AXIS_Y:
                act_reg.act_y = state;
                ret |= adxl355_write_reg(ctx, ADXL355_ACT_EN, (uint8_t*)&act_reg, 1);
                break;
            case ADXL355_AXIS_Z:
                act_reg.act_z = state;
                ret |= adxl355_write_reg(ctx, ADXL355_ACT_EN, (uint8_t*)&act_reg, 1);
                break;
            case ADXL355_AXIS_XYZ:
                act_reg.act_x = state;
                act_reg.act_y = state;
                act_reg.act_z = state;
                ret |= adxl355_write_reg(ctx, ADXL355_ACT_EN, (uint8_t*)&act_reg, 1);
                break;
            default:
                ret = -1;
                break;
        }
    }
    return ret;
}

/*
 * @brief  获取活动阈值
 *
 * @param  ctx        设备句柄
 * @param  threshold  指向存储阈值的指针
 * @retval ret        返回值(返回0->没有错误).
 *
 */
int32_t adxl355_threshold_get(dev_ctx_t* ctx, uint16_t* threshold)
{
    int32_t ret = 0;
    uint8_t tmp[2] = {0};
    ret = adxl355_read_reg(ctx, ADXL355_ACT_THRESH_L, tmp, 1);
    ret |= adxl355_read_reg(ctx, ADXL355_ACT_THRESH_H, tmp+1 , 1);
    *threshold = (((uint16_t)tmp[1]) << 8) | tmp[0];
    return ret;
}

/*
 * @brief  设置活动阈值
 *
 * @param  ctx        设备句柄
 * @param  threshold  阈值
 * @retval ret        返回值(返回0->没有错误).
 *
 */
int32_t adxl355_threshold_set(dev_ctx_t* ctx, uint16_t threshold)
{
    int32_t ret = 0;
    uint8_t tmp[2] = {0};
    tmp[0] = threshold & 0xFF;
    ret = adxl355_write_reg(ctx, ADXL355_ACT_THRESH_L, tmp, 1);
    tmp[1] = threshold >> 8;
    ret |= adxl355_write_reg(ctx, ADXL355_ACT_THRESH_H, tmp, 1);
    return ret;
}

/*
 * @brief  超出阈值计数
 *
 * @param  ctx        设备句柄
 * @param  count      指向计数值的指针
 * @retval ret        返回值(返回0->没有错误).
 *
 */
int32_t adxl355_count_get(dev_ctx_t* ctx, uint8_t* count)
{
    int32_t ret = 0;
    ret = adxl355_read_reg(ctx, ADXL355_ACT_COUNT, count, 1);
    return ret;
}

/*
 * @brief  设置超出阈值计数器
 *
 * @param  ctx        设备句柄
 * @param  count      计数值
 * @retval ret        返回值(返回0->没有错误).
 *
 */
int32_t adxl355_count_set(dev_ctx_t* ctx, uint8_t count)
{
    int32_t ret = 0;
    ret = adxl355_write_reg(ctx, ADXL355_ACT_COUNT, &count, 1);
    return ret;
}

/*
 * @brief  FIFO触发中断的采样数获取（水位线）
 *
 * @param  ctx        设备句柄
 * @param  count      指向采样数的指针
 * @retval ret        返回值(返回0->没有错误).
 *
 */
int32_t adxl355_watermark_get(dev_ctx_t* ctx, uint8_t* sample)
{
    int32_t ret = 0;
    ret = adxl355_read_reg(ctx, ADXL355_FIFO_SAMPLES, sample, 1);
    return ret;
}

/*
 * @brief  设置FIFO触发中断的采样数（水位线）
 *
 * @param  ctx        设备句柄
 * @param  count      要设置的采样数目
 * @retval ret        返回值(返回0->没有错误).
 *
 */
int32_t adxl355_watermark_set(dev_ctx_t* ctx, uint8_t sample)
{
    int32_t ret = 0;
    ret = adxl355_write_reg(ctx, ADXL355_FIFO_SAMPLES, &sample, 1);
    return ret;
}



