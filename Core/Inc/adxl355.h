/*
 * adxl355.h
 *
 *  Created on: Dec 21, 2020
 *      Author: 我是一个酸菜鱼
 */

#ifndef INC_ADXL355_H_
#define INC_ADXL355_H_

#ifdef __cplusplus
  extern "C" {
#endif

/*------------------------------- 头文件包含 -------------------------------*/

#include <stdint.h>
#include <math.h>

/*--------------------- ADXL355 寄存器地址 ---------------------*/

//M_PI
#define ADXL355_DEVID_AD                 0x00
#define ADXL355_DEVID_MST                0x01
#define ADXL355_PARTID                   0x02
#define ADXL355_REVID                    0x03
#define ADXL355_STATUS                   0x04
#define ADXL355_FIFO_ENTRIES             0x05
#define ADXL355_TEMP2                    0x06
#define ADXL355_TEMP1                    0x07
#define ADXL355_XDATA3                   0x08
#define ADXL355_XDATA2                   0x09
#define ADXL355_XDATA1                   0x0A
#define ADXL355_YDATA3                   0x0B
#define ADXL355_YDATA2                   0x0C
#define ADXL355_YDATA1                   0x0D
#define ADXL355_ZDATA3                   0x0E
#define ADXL355_ZDATA2                   0x0F
#define ADXL355_ZDATA1                   0x10
#define ADXL355_FIFO_DATA                0x11
#define ADXL355_OFFSET_X_H               0x1E
#define ADXL355_OFFSET_X_L               0x1F
#define ADXL355_OFFSET_Y_H               0x20
#define ADXL355_OFFSET_Y_L               0x21
#define ADXL355_OFFSET_Z_H               0x22
#define ADXL355_OFFSET_Z_L               0x23
#define ADXL355_ACT_EN                   0x24
#define ADXL355_ACT_THRESH_H             0x25
#define ADXL355_ACT_THRESH_L             0x26
#define ADXL355_ACT_COUNT                0x27
#define ADXL355_FILTER                   0x28
#define ADXL355_FIFO_SAMPLES             0x29
#define ADXL355_INT_MAP                  0x2A
#define ADXL355_SYNC                     0x2B
#define ADXL355_RANGE                    0x2C
#define ADXL355_POWER_CTL                0x2D
#define ADXL355_SELF_TEST                0x2E
#define ADXL355_ADX_RESET                0x2F

/*-------------------- 预定义 -------------------*/
#define PD_DEVID_AD                     0xAD
#define PD_DEVID_MST                    0x1D
#define PD_PARTID                       0xED
#define PD_REVID                        0x01
#define PD_RST                          0x52


#define ADXL355_READ                    0x01
#define ADXL355_WRITE                   0x00

#define ADXL355_TEMP_BIAS               (float)1852.0     /* 25degC 时的截距 */
#define ADXL355_TEMP_SLOPE              (float)-9.05      /* 斜率(LSB/degC) */

#define ADXL355_SCALE_2G                256000.0
#define ADXL355_SCALE_4G                128000.0
#define ADXL355_SCALE_8G                64000.0

/*-------------------------- 函数指针 --------------------------*/
typedef int32_t(*dev_write_reg_ptr)(void*, uint8_t, uint8_t*, uint16_t);
typedef int32_t(*dev_read_reg_ptr)(void*, uint8_t, uint8_t*, uint16_t);

/*-------------------------- 导出类型 -----------------------------*/
typedef struct
{
    void* handle;

    dev_read_reg_ptr read_reg;
    dev_write_reg_ptr write_reg;
} dev_ctx_t;

typedef struct
{
    uint8_t devid_ad;                   /* 模拟设备ID */
    uint8_t devid_mst;                  /* 模拟设备MEMS ID */
    uint8_t partid;                     /* 部件ID */
    uint8_t revid;                      /* 修订ID */
} adxl355_ids_t;

typedef struct
{
    uint8_t data_rdy    : 1;            /* x y z轴数据状态 */
    uint8_t fifo_full   : 1;            /* FIFO满状态 */
    uint8_t fifo_ovr    : 1;            /* FIFO溢出状态 */
    uint8_t activity    : 1;            /* 活动阈值状态 */
    uint8_t nvm_busy    : 1;            /* 非易失性内存繁忙状态 */
    uint8_t not_used    : 3;
} adxl355_reg_status_t;

typedef struct
{
    uint8_t standby     : 1;            /* 待机和测量模式切换 */
    uint8_t temp_off    : 1;            /* 禁用温度处理 */
    uint8_t drdy_off    : 1;            /* 强制DRDY输出0 */
    uint8_t not_used    : 5;
} adxl355_reg_power_t;

typedef enum
{
    ADXL355_MEASURE = 0,
    ADXL355_STANDBY
} adxl355_mode_status;

typedef enum
{
    ADXL355_TEMP_ON = 0,
    ADXL355_TEMP_OFF                    /* 关闭温度处理器 */
} adxl355_temp_status;

typedef enum
{
    ADXL355_DRDY_ON = 0,
    ADXL355_DRDY_OFF
} adxl355_drdy_status;                  /* Set to ADXL355_DRDY_OFF to force the DRDY output to 0 in modes where it is normally signal data ready */

typedef struct
{
    uint32_t raw_axis_x;
    uint32_t raw_axis_y;
    uint32_t raw_axis_z;
    int32_t i32_axis_x;
    int32_t i32_axis_y;
    int32_t i32_axis_z;
    float g_x;
    float g_y;
    float g_z;
} adxl355_axis_data_t;

typedef struct
{
    float angle_x;
    float angle_y;
    float angle_z;
} adxl355_angle_t;

typedef struct
{
    uint8_t range       : 2;            /* 量程控制 */
    uint8_t not_used    : 4;
    uint8_t int_pol     : 1;            /* 中断极性控制 */
    uint8_t i2c_hs      : 1;            /* i2c速度控制 */
} adxl355_reg_range_t;

typedef enum
{
    ADXL355_RANGE_2G = 1,
    ADXL355_RANGE_4G,
    ADXL355_RANGE_8G
} adxl355_range_t;

typedef struct
{
    uint8_t odr_lpf     : 4;            /* 输出速率和低通滤波控制 */
    uint8_t hpf_corner  : 3;            /* 高通滤波-3db控制 */
    uint8_t not_used    : 1;
} adxl355_reg_filter_t;

typedef struct
{
    enum
    {
        HPF_DISABLE = 0,
        CORNER_247_ODR,                 /* 247×10^(-3)×ODR */
        CORNER_62_084_ODR,              /* 62.084×10^(−3)×ODR */
        CORNER_15_545_ODR,              /* 15.545×10^(−3)×ODR */
        CORNER_3_862_ODR,               /* 3.862×10^(−3)×ODR */
        CORNER_0_954_ODR,               /* 0.954×10^(−3)×ODR */
        CORNER_0_238_ODR                /* 0.238×10^(−3)×ODR */
    } hpf;

    enum
    {
        ODR_4000HZ,                     /* 4000Hz   @  1000Hz */
        ODR_2000HZ,                     /* 2000Hz   @  500Hz */
        ODR_1000HZ,                     /* 1000Hz   @  250Hz */
        ODR_500HZ,                      /* 500Hz    @  125Hz */
        ODR_250HZ,                      /* 250Hz    @  62.5Hz */
        ODR_125HZ,                      /* 125Hz    @  31.25Hz */
        ODR_62_5HZ,                     /* 62.5Hz   @  15.625Hz */
        ODR_31_25HZ,                    /* 31.25Hz  @  7.813Hz */
        ODR_15_625HZ,                   /* 15.625Hz @  3.906Hz */
        ODR_7_813HZ,                    /* 7.813Hz  @  1.953Hz */
        ODR_3_906HZ,                    /* 3.906Hz  @  0.977Hz */
    } odr_lpf;

} adxl355_hpf_odr_t;

typedef struct
{
    uint8_t rdy_en1     : 1;            /* 数据就绪中断使能在INT1 */
    uint8_t full_en1    : 1;            /* FIFO满中断使能在INT1 */
    uint8_t ovr_en1     : 1;            /* FIFO溢出中断使能在INT1 */
    uint8_t act_en1     : 1;            /* 活动中断使能在INT1 */
    uint8_t rdy_en2     : 1;            /* 数据就绪中断使能在INT2 */
    uint8_t full_en2    : 1;            /* FIFO满中断使能在INT2 */
    uint8_t ovr_en2     : 1;            /* FIFO溢出中断使能在INT2 */
    uint8_t act_en2     : 1;            /* 活动中断使能在INT2 */
} adxl355_reg_intmap_t;

typedef struct
{
    uint8_t ext_sync    : 2;            /* 外部同步使能控制 */
    uint8_t ext_clk     : 1;            /* 外部始终使能控制 */
    uint8_t not_used    : 5;
} adxl355_reg_sync_t;

typedef enum
{
    ADXL355_DISABLE = 0,
    ADXL355_ENABLE
} adxl355_state_t;

typedef enum
{
    ADXL355_INTER_SYNC = 0,             /* 内部同步 */
    ADXL355_EXTER_SYNC,                 /* 外部同步，无插值滤波器 */
    ADXL355_EXTER_SYNC_FILTER           /* 外部同步，有插值滤波器 */
} adxl355_sync_t;

typedef struct
{
    uint8_t st1     : 1;                /* 使能自测模式 */
    uint8_t st2     : 1;                /* 强制使能自测 */
    uint8_t not_used: 6;
} adxl355_reg_st_t;

typedef struct
{
    enum
    {
        ADXL355_ST_DISABLE = 0,
        ADXL355_ST_ENABLE
    }st1;
    enum
    {
        ADXL355_FORCEST_DISABLE = 0,
        ADXL355_FORCEST_ENABLE
    }st2;

} adxl355_st_state_t;

typedef struct
{
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
} adxl355_offset_val_t;

typedef enum
{
    ADXL355_AXIS_X = 0,
    ADXL355_AXIS_Y,
    ADXL355_AXIS_Z,
    ADXL355_AXIS_XYZ
} adxl355_axis_t;

typedef struct
{
    uint8_t act_x       : 1;            /* x轴活动算法使能控制 */
    uint8_t act_y       : 1;            /* y轴活动算法使能控制 */
    uint8_t act_z       : 1;            /* z轴活动算法使能控制 */
    uint8_t not_used    : 5;
} adxl355_reg_act_t;



/*------------------------------ 函数 ------------------------------*/
int32_t adxl355_read_reg(dev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len);
int32_t adxl355_write_reg(dev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len);

int32_t adxl355_ids_get(dev_ctx_t* ctx, adxl355_ids_t *ids);
int32_t adxl355_status_get(dev_ctx_t* ctx, adxl355_reg_status_t* status);
int32_t adxl355_fifo_entries_get(dev_ctx_t* ctx, uint8_t* entries);
int32_t adxl355_temperature_get(dev_ctx_t* ctx, float* temperature);
int32_t adxl355_mode_get(dev_ctx_t* ctx, adxl355_mode_status* mode);
int32_t adxl355_mode_set(dev_ctx_t* ctx, adxl355_mode_status mode);
int32_t adxl355_soft_reset(dev_ctx_t* ctx);
int32_t adxl355_temp_off_get(dev_ctx_t* ctx, adxl355_temp_status* status);
int32_t adxl355_temp_off_set(dev_ctx_t* ctx, adxl355_temp_status status);
int32_t adxl355_drdy_off_get(dev_ctx_t* ctx, adxl355_drdy_status* status);
int32_t adxl355_drdy_off_set(dev_ctx_t* ctx, adxl355_drdy_status status);
int32_t adxl355_axis_get(dev_ctx_t* ctx, adxl355_axis_data_t* axis_data, adxl355_range_t range);
int32_t adxl355_angle_get(adxl355_axis_data_t axis_data, adxl355_angle_t* angles);
int32_t adxl355_range_get(dev_ctx_t* ctx, adxl355_range_t* range);
int32_t adxl355_range_set(dev_ctx_t* ctx, adxl355_range_t range);
int32_t adxl355_hpf_odr_get(dev_ctx_t* ctx, adxl355_hpf_odr_t* hpf_odr);
int32_t adxl355_hpf_odr_set(dev_ctx_t* ctx, adxl355_hpf_odr_t hpf_odr);
int32_t adxl355_intmap_get(dev_ctx_t* ctx, adxl355_reg_intmap_t* int_map);
int32_t adxl355_intmap_set(dev_ctx_t* ctx, adxl355_reg_intmap_t int_map);
int32_t adxl355_exclk_get(dev_ctx_t* ctx, adxl355_state_t* state);
int32_t adxl355_exclk_set(dev_ctx_t* ctx, adxl355_state_t state);
int32_t adxl355_sync_get(dev_ctx_t* ctx, adxl355_sync_t* sync);
int32_t adxl355_sync_set(dev_ctx_t* ctx, adxl355_sync_t sync);
int32_t adxl355_st_get(dev_ctx_t* ctx, adxl355_st_state_t* st_state);
int32_t adxl355_st_set(dev_ctx_t* ctx,adxl355_st_state_t st_state);
int32_t adxl355_offset_get(dev_ctx_t* ctx, adxl355_offset_val_t* offset, adxl355_axis_t axis);
int32_t adxl355_offset_set(dev_ctx_t* ctx, adxl355_offset_val_t offset, adxl355_axis_t axis);
int32_t adxl355_act_get(dev_ctx_t* ctx, adxl355_state_t* state, adxl355_axis_t axis);
int32_t adxl355_act_set(dev_ctx_t* ctx, adxl355_state_t state, adxl355_axis_t axis);
int32_t adxl355_threshold_get(dev_ctx_t* ctx, uint16_t* threshold);
int32_t adxl355_threshold_set(dev_ctx_t* ctx, uint16_t threshold);
int32_t adxl355_count_get(dev_ctx_t* ctx, uint8_t* count);
int32_t adxl355_count_set(dev_ctx_t* ctx, uint8_t count);
int32_t adxl355_watermark_get(dev_ctx_t* ctx, uint8_t* sample);
int32_t adxl355_watermark_set(dev_ctx_t* ctx, uint8_t sample);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADXL355_H_ */
