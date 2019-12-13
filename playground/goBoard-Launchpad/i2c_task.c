//*****************************************************************************
//
// i2c_task.h - Task for I2C transactions with peripherals
//
// Paul Schneider 2019
// Reference and structure based on led_task of TivaWare Sample projects
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"

#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "i2c_task.h"

#include "periph_i2c.h"
#include "interface_pca9685.h"
//#include "BNO055_driver/bno055.h"


#define CTRL_F_FWD 0
#define CTRL_F_REV 4

#define CTRL_F_UP 0
#define CTRL_R_UP 4
#define CTRL_F_DN 0
#define CTRL_R_DN 4

#define I2CTASKSTACKSIZE        128         // Stack size in words

// Message queue size for I2C transaction (read/write) requests
#define I2C_ITEM_SIZE           sizeof(uint8_t) * 5
#define I2C_QUEUE_SIZE          32

//*********** TODO
//extern struct bno055_t bno055;
//extern BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055);
//*********************

xQueueHandle g_pI2CQueue;

struct pca9685_t pwm;

static void I2CTask(void *pvParameters) {
    vTaskDelay(1000);
    UARTprintf("\nI2C First Tick!\n");
    portTickType ui32WakeTime = xTaskGetTickCount();

    uint8_t cmd_angle_front = 90;
    uint8_t cmd_angle_rear = 90;

    uint8_t command_message[5];

    // FR 90 deg down = 700
    // FR straight  = 1280
    // FR 90 deg up = 1820

    // FL 90 deg down  = 1780
    // FL straight  = 1170
    // FL 90 deg up 640


    while (1) {
        UARTprintf("\nI2C tick!\n");

        if(xQueueReceive(g_pI2CQueue, &command_message, 0) == pdPASS) {
            // tx = {CTRL_ACCEL, CTRL_UP, CTRL_DN, CHECKSUM, 0x00};

            if (command_message[1]) {
                if (command_message[1] & (1 << CTRL_F_UP)) {
                    if (cmd_angle_front < 180) cmd_angle_front += 5;
                    UARTprintf("FRONT UP\n");
                }
                if (command_message[1] & (1 << CTRL_R_UP)) {
                    if (cmd_angle_rear < 180) cmd_angle_rear += 5;
                    UARTprintf("REAR UP\n");
                }
            }
            if (command_message[2]) {
                if (command_message[2] & (1 << CTRL_F_DN)) {
                    if (cmd_angle_front > 0) cmd_angle_front -= 5;
                    UARTprintf("FRONT DOWN\n");
                }
                if (command_message[2] & (1 << CTRL_R_DN)) {
                    if (cmd_angle_rear > 0) cmd_angle_rear -= 5;
                    UARTprintf("REAR DOWN\n");
                }
            }
        }


        writeAngle(&pwm, FRONT_LEFT, cmd_angle_front);
        writeAngle(&pwm, FRONT_RIGHT, cmd_angle_front);
        writeAngle(&pwm, REAR_LEFT, cmd_angle_rear);
        writeAngle(&pwm, REAR_RIGHT, cmd_angle_rear);
//      UARTprintf("Writing: %d\n", tmp_angle);

        vTaskDelay(10);
    }


}

//sends an array of data via I2C to the specified slave







int I2CTaskInit() {

    initI2C(I2C1_BASE);

    pca9685Init(&pwm, I2C1_BASE, 0x40);

    g_servo_map.servo_offset[FRONT_LEFT] = 700;
    g_servo_map.servo_offset[FRONT_RIGHT] = 640;

    g_servo_map.servo_offset[REAR_LEFT] = 640;
    g_servo_map.servo_offset[REAR_RIGHT] = 700;

    // Dir ? --> CCW rotate (rotate up for FR RL)
    g_servo_map.dir= 0x00;
    g_servo_map.dir |= (0 << FRONT_LEFT);
    g_servo_map.dir |= (1 << FRONT_RIGHT);
    g_servo_map.dir |= (1 << REAR_LEFT);
    g_servo_map.dir |= (0 << REAR_RIGHT);

    g_servo_map.servo_channel[FRONT_LEFT] = 0;
    g_servo_map.servo_channel[FRONT_RIGHT] = 2;
    g_servo_map.servo_channel[REAR_LEFT] = 4;
    g_servo_map.servo_channel[REAR_RIGHT] = 6;

    g_servo_map.servo_upper_limit = 2100;
    g_servo_map.servo_lower_limit = 500;


    g_pI2CQueue = xQueueCreate(I2C_QUEUE_SIZE, I2C_ITEM_SIZE);

    if(xTaskCreate(I2CTask, (const portCHAR *)"I2C", I2CTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_I2C_TASK, NULL) != pdTRUE)
    {
        return 1;
    }
    UARTprintf("Task Init Complete\n");

    return 0;
}
