//*****************************************************************************
//
// led_task.h - Basic I2C Transaction with BNO055 sensor
//
// Paul Schneider
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"

#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


//*****************************************************************************
//
// The stack size for the I2C toggle task.
//
//*****************************************************************************
#define I2CTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// The item size and queue size for the I2C message queue.
//
//*****************************************************************************
#define I2C_ITEM_SIZE           sizeof(uint8_t)
#define I2C_QUEUE_SIZE          5

//*****************************************************************************
//
// The queue that holds messages sent to the I2C task.
//
//*****************************************************************************
xQueueHandle g_pI2CQueue;

static void I2CTask(void *pvParameters) {
    UARTprintf("\nI2C First Tick!\n");
    portTickType ui32WakeTime = xTaskGetTickCount();

    while (1) {
        UARTprintf("\nI2C tick!\n");
        vTaskDelay(1000);
    }


}

int I2CTaskInit() {
    UARTprintf("\nInitializing I2C Task\n");

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);

    UARTprintf("\nNI2C Config complete\n");

    uint32_t ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_XTAL_25MHZ |
    SYSCTL_CFG_VCO_480), 120000000);

    SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    UARTprintf("\nNI2C Config complete\n");

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2));

    I2CMasterInitExpClk(I2C2_BASE, ui32SysClock, false);
    UARTprintf("\nNI2C Config complete\n");

    I2CMasterIntEnableEx(I2C2_BASE, (I2C_MASTER_INT_ARB_LOST |
    I2C_MASTER_INT_STOP | I2C_MASTER_INT_NACK |
    I2C_MASTER_INT_TIMEOUT | I2C_MASTER_INT_DATA));
    UARTprintf("\nNI2C Config complete\n");

    extern void IntHandler(void);
    //
    // Register the interrupt handler function for UART 0.
    //
    IntRegister(INT_I2C2, IntHandler);
    //



    IntEnable(INT_I2C2);
    UARTprintf("\nNI2C Config complete\n");



    g_pI2CQueue = xQueueCreate(I2C_QUEUE_SIZE, I2C_ITEM_SIZE);

    if(xTaskCreate(I2CTask, (const portCHAR *)"I2C", I2CTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_I2C_TASK, NULL) != pdTRUE)
    {
        return 1;
    }

    return 0;
}
