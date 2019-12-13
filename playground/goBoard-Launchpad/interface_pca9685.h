#ifndef _INTERFACE_PCA9685_
#define _INTERFACE_PCA9685_

#include "pca9685.h"
#include <stdbool.h>
#include <stdint.h>

#include "i2c_task.h"

struct pca9685_t {
    uint32_t i2c_base;
    uint8_t l_addr;
    uint8_t l_mode1;
    uint8_t l_mode2;
    uint8_t l_allcalladr;
    uint8_t l_suabdr1;
    uint8_t l_suabdr2;
    uint8_t l_suabdr3;
    uint8_t l_prescale;
} g_pca9685;

uint8_t pca9685Init(struct pca9685_t *pca9685, uint32_t i2c_base, uint8_t dev_addr);

uint8_t writePwm(struct pca9685_t *pca9685, uint8_t led_channel, uint16_t on_time, uint16_t off_time);

void writeAngle(struct pca9685_t *pca9685, enum SERVO_IDX idx, uint16_t degrees);

#endif // _INTERFACE_PCA9685_
