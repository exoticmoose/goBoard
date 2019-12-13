//*****************************************************************************
//
// gpio_task.c - Task for I2C transactions with peripherals
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

#include "driverlib/interrupt.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"

#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define F_ENA_PORT  GPIO_PORTC_BASE
#define F_ENA_PIN   GPIO_PIN_5
#define F_IN1_PORT  GPIO_PORTC_BASE
#define F_IN1_PIN   GPIO_PIN_6
#define F_IN2_PORT  GPIO_PORTC_BASE
#define F_IN2_PIN   GPIO_PIN_7
#define F_IN3_PORT  GPIO_PORTD_BASE
#define F_IN3_PIN   GPIO_PIN_6
#define F_IN4_PORT  GPIO_PORTD_BASE
#define F_IN4_PIN   GPIO_PIN_7
#define F_ENB_PORT  GPIO_PORTF_BASE
#define F_ENB_PIN   GPIO_PIN_4

#define R_ENA_PORT  GPIO_PORTE_BASE
#define R_ENA_PIN   GPIO_PIN_0
#define R_IN1_PORT  GPIO_PORTE_BASE
#define R_IN1_PIN   GPIO_PIN_1
#define R_IN2_PORT  GPIO_PORTE_BASE
#define R_IN2_PIN   GPIO_PIN_2
#define R_IN3_PORT  GPIO_PORTE_BASE
#define R_IN3_PIN   GPIO_PIN_3
#define R_IN4_PORT  GPIO_PORTE_BASE
#define R_IN4_PIN   GPIO_PIN_4
#define R_ENB_PORT  GPIO_PORTE_BASE
#define R_ENB_PIN   GPIO_PIN_5

#define CTRL_F_FWD 0
#define CTRL_F_REV 4


#define GPIOTASKSTACKSIZE        128         // Stack size in words

// Message queue size for SSI transaction (read/write) requests
#define GPIO_ITEM_SIZE           sizeof(uint8_t) * 5
#define GPIO_QUEUE_SIZE          32



xQueueHandle g_pGPIOQueue;
void revDriveEnable() {
    GPIOPinWrite(F_ENA_PORT, F_ENA_PIN, F_ENA_PIN);
    GPIOPinWrite(F_IN1_PORT, F_IN1_PIN, F_IN1_PIN);
    GPIOPinWrite(F_IN2_PORT, F_IN2_PIN, 0);
    GPIOPinWrite(F_IN3_PORT, F_IN3_PIN, F_IN3_PIN);
    GPIOPinWrite(F_IN4_PORT, F_IN4_PIN, 0);
    GPIOPinWrite(F_ENB_PORT, F_ENB_PIN, F_ENB_PIN);

    GPIOPinWrite(R_ENA_PORT, R_ENA_PIN, R_ENA_PIN);
    GPIOPinWrite(R_IN1_PORT, R_IN1_PIN, 0);
    GPIOPinWrite(R_IN2_PORT, R_IN2_PIN, R_IN2_PIN);
    GPIOPinWrite(R_IN3_PORT, R_IN3_PIN, 0);
    GPIOPinWrite(R_IN4_PORT, R_IN4_PIN, R_IN4_PIN);
    GPIOPinWrite(R_ENB_PORT, R_ENB_PIN, R_ENB_PIN);
}

void fwdDriveEnable() {
    GPIOPinWrite(F_ENA_PORT, F_ENA_PIN, F_ENA_PIN);
    GPIOPinWrite(F_IN1_PORT, F_IN1_PIN, 0);
    GPIOPinWrite(F_IN2_PORT, F_IN2_PIN, F_IN2_PIN);
    GPIOPinWrite(F_IN3_PORT, F_IN3_PIN, 0);
    GPIOPinWrite(F_IN4_PORT, F_IN4_PIN, F_IN4_PIN);
    GPIOPinWrite(F_ENB_PORT, F_ENB_PIN, F_ENB_PIN);

    GPIOPinWrite(R_ENA_PORT, R_ENA_PIN, R_ENA_PIN);
    GPIOPinWrite(R_IN1_PORT, R_IN1_PIN, R_IN1_PIN);
    GPIOPinWrite(R_IN2_PORT, R_IN2_PIN, 0);
    GPIOPinWrite(R_IN3_PORT, R_IN3_PIN, R_IN3_PIN);
    GPIOPinWrite(R_IN4_PORT, R_IN4_PIN, 0);
    GPIOPinWrite(R_ENB_PORT, R_ENB_PIN, R_ENB_PIN);
}

void stopDrive() {
    GPIOPinWrite(F_ENA_PORT, F_ENA_PIN, 0);
    GPIOPinWrite(F_IN1_PORT, F_IN1_PIN, 0);
    GPIOPinWrite(F_IN2_PORT, F_IN2_PIN, 0);
    GPIOPinWrite(F_IN3_PORT, F_IN3_PIN, 0);
    GPIOPinWrite(F_IN4_PORT, F_IN4_PIN, 0);
    GPIOPinWrite(F_ENB_PORT, F_ENB_PIN, 0);

    GPIOPinWrite(R_ENA_PORT, R_ENA_PIN, 0);
    GPIOPinWrite(R_IN1_PORT, R_IN1_PIN, 0);
    GPIOPinWrite(R_IN2_PORT, R_IN2_PIN, 0);
    GPIOPinWrite(R_IN3_PORT, R_IN3_PIN, 0);
    GPIOPinWrite(R_IN4_PORT, R_IN4_PIN, 0);
    GPIOPinWrite(R_ENB_PORT, R_ENB_PIN, 0);
}

static void GPIOTask(void *pvParameters) {
    UARTprintf("\nGPIO First Tick!\n");
    portTickType ui32WakeTime = xTaskGetTickCount();
    uint8_t command_message[5];


    while (1) {
        UARTprintf("\nGPIO tick!\n");
        if(xQueueReceive(g_pGPIOQueue, &command_message, 0) == pdPASS) {
                    // tx = {CTRL_ACCEL, CTRL_UP, CTRL_DN, CHECKSUM, 0x00};

                    if (command_message[0]) {
                        if (command_message[0] & (1 << CTRL_F_FWD)) {
                            // fwd precedence
                            fwdDriveEnable();
                            UARTprintf("FORWARD DRIVE\n");
                        }
                        else if (command_message[0] & (1 << CTRL_F_REV)) {
                            revDriveEnable();
                            UARTprintf("REVERSE DRIVE\n");
                        }

                    }
                    else {
                                                stopDrive();
                                            }

                }
        vTaskDelay(50);
    }


}



int GPIOTaskInit() {


    // Enable peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIOPinTypeGPIOOutput(F_ENA_PORT, F_ENA_PIN);
    GPIOPinTypeGPIOOutput(F_IN1_PORT, F_IN1_PIN);
    GPIOPinTypeGPIOOutput(F_IN2_PORT, F_IN2_PIN);
    GPIOPinTypeGPIOOutput(F_IN3_PORT, F_IN3_PIN);
    GPIOPinTypeGPIOOutput(F_IN4_PORT, F_IN4_PIN);
    GPIOPinTypeGPIOOutput(F_ENB_PORT, F_ENB_PIN);

    GPIOPinTypeGPIOOutput(R_ENA_PORT, R_ENA_PIN);
    GPIOPinTypeGPIOOutput(R_IN1_PORT, R_IN1_PIN);
    GPIOPinTypeGPIOOutput(R_IN2_PORT, R_IN2_PIN);
    GPIOPinTypeGPIOOutput(R_IN3_PORT, R_IN3_PIN);
    GPIOPinTypeGPIOOutput(R_IN4_PORT, R_IN4_PIN);
    GPIOPinTypeGPIOOutput(R_ENB_PORT, R_ENB_PIN);

    GPIOPinWrite(F_ENA_PORT, F_ENA_PIN, 0);
    GPIOPinWrite(F_IN1_PORT, F_IN1_PIN, 0);
    GPIOPinWrite(F_IN2_PORT, F_IN2_PIN, 0);
    GPIOPinWrite(F_IN3_PORT, F_IN3_PIN, 0);
    GPIOPinWrite(F_IN4_PORT, F_IN4_PIN, 0);
    GPIOPinWrite(F_ENB_PORT, F_ENB_PIN, 0);

    GPIOPinWrite(R_ENA_PORT, R_ENA_PIN, 0);
    GPIOPinWrite(R_IN1_PORT, R_IN1_PIN, 0);
    GPIOPinWrite(R_IN2_PORT, R_IN2_PIN, 0);
    GPIOPinWrite(R_IN3_PORT, R_IN3_PIN, 0);
    GPIOPinWrite(R_IN4_PORT, R_IN4_PIN, 0);
    GPIOPinWrite(R_ENB_PORT, R_ENB_PIN, 0);



    g_pGPIOQueue = xQueueCreate(GPIO_QUEUE_SIZE, GPIO_ITEM_SIZE);

    if(xTaskCreate(GPIOTask, (const portCHAR *)"GPIO", GPIOTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SSI_TASK, NULL) != pdTRUE)
    {
        return 1;
    }
    UARTprintf("GPIO Task Init Complete\n");

    return 0;
}
