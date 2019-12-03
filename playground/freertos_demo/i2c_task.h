//*****************************************************************************
//
// led_task.h - Prototypes for the LED task.
//
// Paul Schneider
// Reference and structure based on led_task of TivaWare Sample projects
//
//*****************************************************************************

#ifndef __I2C_TASK_H__
#define __I2C_TASK_H__

//*****************************************************************************
//
// Prototypes for the LED task.
//
//*****************************************************************************
extern int I2CTaskInit();
void I2CSendString(uint32_t slave_addr, char array[]);

#endif // __I2C_TASK_H__
