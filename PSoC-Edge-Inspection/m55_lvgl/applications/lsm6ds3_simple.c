// =====================================================
//  lsm6ds3_simple.c
// =====================================================

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drivers/lsm6ds3tr/lsm6ds3tr-c_reg.h"
#include "drivers/lsm6ds3tr/lsm6ds3tr-c_port.h"

static stmdev_ctx_t lsm6ds3_ctx;
static uint8_t lsm6ds3_ready = 0;

// =====================================================
//  I2C 读写回调
// =====================================================
static int32_t lsm6ds3_i2c_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    struct rt_i2c_msg msgs[2];
    struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)handle;

    msgs[0].addr = 0x6A;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = 0x6A;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = bufp;
    msgs[1].len = len;

    if (rt_i2c_transfer(i2c_bus, msgs, 2) == 2) {
        return 0;
    }
    return -1;
}

static int32_t lsm6ds3_i2c_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    struct rt_i2c_msg msg;
    struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)handle;
    uint8_t send_buf[32];

    if (len > 30) return -1;

    send_buf[0] = reg;
    for (int i = 0; i < len; i++) {
        send_buf[i + 1] = bufp[i];
    }

    msg.addr = 0x6A;
    msg.flags = RT_I2C_WR;
    msg.buf = send_buf;
    msg.len = len + 1;

    if (rt_i2c_transfer(i2c_bus, &msg, 1) == 1) {
        return 0;
    }
    return -1;
}

// =====================================================
//  初始化 LSM6DS3
// =====================================================
int lsm6ds3_simple_init(void)
{
    struct rt_i2c_bus_device *i2c_bus;

    i2c_bus = rt_i2c_bus_device_find("i2c0");
    if (i2c_bus == RT_NULL) {
        rt_kprintf("LSM6DS3: i2c0 not found!\n");
        return -1;
    }

    lsm6ds3_ctx.handle = i2c_bus;
    lsm6ds3_ctx.read_reg = lsm6ds3_i2c_read;
    lsm6ds3_ctx.write_reg = lsm6ds3_i2c_write;
    lsm6ds3_ctx.mdelay = (stmdev_mdelay_ptr)rt_thread_mdelay;  // 修复类型警告

    // 校验 WHO_AM_I（修复函数名）
    uint8_t whoami = 0;
    lsm6ds3tr_c_device_id_get(&lsm6ds3_ctx, &whoami);
    if (whoami != LSM6DS3TR_C_ID) {
        rt_kprintf("LSM6DS3: WHO_AM_I error (0x%02X, expected 0x6A)\n", whoami);
        return -1;
    }

    // 软件复位
    lsm6ds3tr_c_reset_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    uint8_t rst = 1;
    while (rst) {
        lsm6ds3tr_c_reset_get(&lsm6ds3_ctx, &rst);
        rt_thread_mdelay(10);
    }

    // 配置: 52Hz, ±2g
    lsm6ds3tr_c_xl_data_rate_set(&lsm6ds3_ctx, LSM6DS3TR_C_XL_ODR_52Hz);
    lsm6ds3tr_c_xl_full_scale_set(&lsm6ds3_ctx, LSM6DS3TR_C_2g);

    lsm6ds3_ready = 1;
    rt_kprintf("LSM6DS3: init OK (0x6A, 52Hz)\n");
    return 0;
}

// =====================================================
//  读取加速度 (单位: mg)
// =====================================================
int lsm6ds3_simple_read_accel(int16_t *x, int16_t *y, int16_t *z)
{
    int16_t data[3];
    if (!lsm6ds3_ready) {
        *x = *y = *z = 0;
        return -1;
    }
    lsm6ds3tr_c_acceleration_raw_get(&lsm6ds3_ctx, data);
    *x = data[0];
    *y = data[1];
    *z = data[2];
    return 0;
}

// =====================================================
//  读取温度 (原始值)
// =====================================================
int lsm6ds3_simple_read_temp(int16_t *temp)
{
    if (!lsm6ds3_ready) {
        *temp = 0;
        return -1;
    }
    lsm6ds3tr_c_temperature_raw_get(&lsm6ds3_ctx, temp);
    return 0;
}
