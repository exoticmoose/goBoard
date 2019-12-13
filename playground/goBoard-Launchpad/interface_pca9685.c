

#include <stdbool.h>
#include <stdint.h>

#include "interface_pca9685.h"
#include "periph_i2c.h"
#include "utils/uartstdio.h"

#include "i2c_task.h"

extern struct servo_map_t g_servo_map;

uint8_t pca9685Init(struct pca9685_t* pca9685, uint32_t i2c_base, uint8_t dev_addr)
{
    pca9685->i2c_base = i2c_base;
    pca9685->l_addr = dev_addr;

    // Device initialization for basic servo control

    pca9685->l_addr = 0x40;      // TODO definition
    pca9685->l_mode1 = ALLCALL;  // TODO functionize
    pca9685->l_mode2 = OUTDRV;   // TODO functionize
    pca9685->l_prescale = 0x14;  // TODO functionize

    uint8_t buf[2];

    buf[0] = MODE1_ADDR;
    buf[1] = pca9685->l_mode1;
    writeI2C(pca9685->i2c_base, pca9685->l_addr, buf, 2);

    readI2C(pca9685->i2c_base, pca9685->l_addr, MODE1_ADDR, buf, 1);

    buf[0] = MODE2_ADDR;
    buf[1] = pca9685->l_mode2;
    writeI2C(pca9685->i2c_base, pca9685->l_addr, buf, 2);

    readI2C(pca9685->i2c_base, pca9685->l_addr, MODE2_ADDR, buf, 1);

    // prescale reg = ~406.9Hz, T = 2.457599ms, delt = 0.59999999999999995us ~= 0.6us
    buf[0] = PRESCALE_ADDR;
    buf[1] = pca9685->l_prescale;
    writeI2C(pca9685->i2c_base, pca9685->l_addr, buf, 2);

    readI2C(pca9685->i2c_base, pca9685->l_addr, PRESCALE_ADDR, buf, 1);

    return 0; // TODO fault checks
}

uint8_t writePwm(struct pca9685_t *pca9685, uint8_t led_channel, uint16_t on_time, uint16_t off_time) {

    uint16_t HI_TIME_TRUNC = on_time >= 0x0FFF ? 0x1FFF : on_time & 0x0FFF;
    uint16_t LO_TIME_TRUNC = off_time >= 0x0FFF ? 0x1FFF : off_time & 0x0FFF;

    uint8_t addr[4];
    addr[0] = 0x06 + led_channel * 4;
    addr[1] = 0x07 + led_channel * 4;
    addr[2] = 0x08 + led_channel * 4;
    addr[3] = 0x09 + led_channel * 4;

    uint8_t buffer[8];

    //UARTprintf("PCA9685 Write ch. %d on/off: %d / %d \n", led_channel, HI_TIME_TRUNC, LO_TIME_TRUNC);

    buffer[0] = addr[0];
    buffer[1] = HI_TIME_TRUNC & 0xFF;
    writeI2C(pca9685->i2c_base, pca9685->l_addr, buffer, 2);

    buffer[0] = addr[1];
    buffer[1] = (HI_TIME_TRUNC >> 8) & 0xFF;
    writeI2C(pca9685->i2c_base, pca9685->l_addr, buffer, 2);

    buffer[0] = addr[2];
    buffer[1] = LO_TIME_TRUNC & 0xFF;
    writeI2C(pca9685->i2c_base, pca9685->l_addr, buffer, 2);

    buffer[0] = addr[3];
    buffer[1] = (LO_TIME_TRUNC >> 8) & 0xFF;
    writeI2C(pca9685->i2c_base, pca9685->l_addr, buffer, 2);

    return 0; //TODO
}

#define FL_TRUNC_UPPER

void writeAngle(struct pca9685_t *pca9685, enum SERVO_IDX idx, uint16_t degrees) {
    uint16_t off_count = g_servo_map.servo_offset[idx];
//    /uint16_t time_goal = 0.0;
    bool dir = g_servo_map.dir & (1 << idx);

    off_count = dir ? off_count + (degrees * 19 / 3) : off_count + ((180 - degrees) * 19 / 3); // TODO calibration auto
    //off_count = time_goal;// * DELT_TIME;
    if (off_count > g_servo_map.servo_lower_limit && off_count < g_servo_map.servo_upper_limit) {
        writePwm(pca9685, g_servo_map.servo_channel[idx], 0, off_count);
    }

}
