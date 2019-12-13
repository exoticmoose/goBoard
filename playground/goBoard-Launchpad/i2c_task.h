//*****************************************************************************
//
// i2c_task.h - Prototypes for the I2C management task.
//
// Paul Schneider 2019
// Reference and structure based on led_task of TivaWare Sample projects
//
//*****************************************************************************

#ifndef __I2C_TASK_H__
#define __I2C_TASK_H__

struct servo_map_t {

    uint16_t servo_offset[16];
    uint8_t servo_channel[16];
    uint16_t servo_upper_limit;
    uint16_t servo_lower_limit;
    uint16_t dir; // one bit per dir.

} g_servo_map;

enum SERVO_IDX {
    FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
};
//*****************************************************************************
//
// Prototypes for the LED task.
//
//*****************************************************************************
extern int I2CTaskInit();

#endif // __I2C_TASK_H__
