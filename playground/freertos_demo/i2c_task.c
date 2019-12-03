//*****************************************************************************
//
// led_task.h - Basic I2C Transaction with BNO055 sensor
//
// Paul Schneider
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

#include "BNO055_driver/bno055.h"


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

//*****************************************************************************
//
// BNO055 Driver interface requirements
//
//*****************************************************************************
struct bno055_t goBNO055;


extern s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
extern s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
extern void BNO055_delay_msek(u32 msek);

extern s32 bno055_data_readout_template(void);

extern uint32_t i2c1_read(uint32_t slave_addr, uint8_t reg_addr, uint8_t *array, uint8_t cnt);
extern uint32_t i2c1_write(uint32_t slave_addr, uint8_t array[], uint8_t cnt);


//*****************************************************************************
//
// Main task and initializer
//
//*****************************************************************************

static void I2CTask(void *pvParameters) {
    UARTprintf("\nI2C First Tick!\n");
    portTickType ui32WakeTime = xTaskGetTickCount();

    //s32 BNO055_iERROR = BNO055_INIT_VALUE;
    //struct bno055_accel_t accel;



    while (1) {
        UARTprintf("\nI2C tick!\n");
        vTaskDelay(10);


        // Set I2C slave addr for TX
        I2CMasterSlaveAddrSet(I2C1_BASE, 0xAA, false);

        // Register address into MDR
        I2CMasterDataPut(I2C1_BASE, 0x55);

        // Initialize TX transaction, wait to finish
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
        while(I2CMasterBusy(I2C1_BASE));

//
//        //BNO055_iERROR = bno055_read_accel_xyz(&accel);
//        if (BNO055_iERROR) {
//            UARTprintf("Errr reading accel xyz");
//        }
//        else {
//            UARTprintf("X = %i, Y = %i, Z = %i\n", accel.x, accel.y, accel.z);
//        }

        vTaskDelay(100);
    }


}

//sends an array of data via I2C to the specified slave

extern struct bno055_t bno055;
extern BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055);

void writeLed(uint8_t led_channel, uint16_t on_time, uint16_t off_time) {

    uint16_t HI_TIME_TRUNC = on_time >= 0x0FFF ? 0x1FFF : on_time & 0x0FFF;
    uint16_t LO_TIME_TRUNC = off_time >= 0x0FFF ? 0x1FFF : off_time & 0x0FFF;

    uint8_t addr[4];
    addr[0] = 0x06 + led_channel * 4;
    addr[1] = 0x07 + led_channel * 4;
    addr[2] = 0x08 + led_channel * 4;
    addr[3] = 0x09 + led_channel * 4;

    uint8_t buffer[2];

    buffer[0] = addr[0];
    buffer[1] = HI_TIME_TRUNC & 0xFF;
    i2c1_write(0x40, buffer, 2);

    buffer[0] = addr[1];
    buffer[1] = (HI_TIME_TRUNC >> 8) & 0xFF;
    i2c1_write(0x40, buffer, 2);

    buffer[0] = addr[2];
    buffer[1] = LO_TIME_TRUNC & 0xFF;
    i2c1_write(0x40, buffer, 2);

    buffer[0] = addr[3];
    buffer[1] = (LO_TIME_TRUNC >> 8) & 0xFF;
    i2c1_write(0x40, buffer, 2);

}

#define DELT_TIME 6 / 10 // 0.6us per LSB off_count @ 407Hz prescaler
#define TIME_OFFSET 920
void writeAngle(uint8_t channel, uint16_t degrees) {
    uint16_t off_count = 0;
    uint16_t time_goal = 0.0;

    time_goal = TIME_OFFSET + degrees * 10;
    off_count = time_goal;// * DELT_TIME;

    writeLed(channel, 0, off_count);
}


int I2CTaskInit() {
    UARTprintf("\nInitializing I2C Task\n");

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    //enable I2C module 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);





    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

    //enable GPIO peripheral that contains I2C 1 - MOVED UP
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);


    // Set I2C slave addr for TX
    I2CMasterSlaveAddrSet(I2C1_BASE, 0xAA, false);

    // Register address into MDR
    I2CMasterDataPut(I2C1_BASE, 0x55);

    // Initialize TX transaction, wait to finish
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C1_BASE));

    //IntEnable(INT_I2C2);
    UARTprintf("I2C Config Complete\n");

    uint8_t tmp[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF};
    uint8_t buffer[8] = {0};
    //bno055_init(bno055);


    i2c1_read(0x40, 0x00, tmp, 1);

//    buffer[0] = 0x00;
//    buffer[1] = 0b00010000;
//
//    i2c1_write(0x40, &buffer, 2);
//    i2c1_read(0x40, 0x00, tmp, 1);
//
//    buffer[0] = 0x00;
//    buffer[1] = 0xAA;
//
//    i2c1_write(0x40, &buffer, 2);
//    i2c1_read(0x40, 0x00, tmp, 1);


    //init mode

    // prescale reg = ~406.9Hz, T = 2.457599ms, delt = 0.59999999999999995us ~= 0.6us
    buffer[0] = 0xFE;
    buffer[1] = 0x14;
    i2c1_write(0x40, buffer, 2);

    // mode 1 reg
    buffer[0] = 0x00;
    buffer[1] = 0b00000001;
    i2c1_write(0x40, buffer, 2);

    // mode 2 reg
    buffer[0] = 0x01;
    buffer[1] = 0b00000100;
    i2c1_write(0x40, buffer, 2);


    while (1) {
        for (int i = 0; i < 270; i += 1) {
            writeAngle(1, i);
            UARTprintf("led1 angle: %d\n", i);
            SysCtlDelay(25 * (SysCtlClockGet() / 3 / 1000));
        }

        UARTprintf("Pause A.\n");
        SysCtlDelay(1000 * (SysCtlClockGet() / 3 / 1000));

        for (int i = 270; i > 0 ; i -= 1) {

            writeAngle(1, i);
            //UARTprintf("led0 on: %d to 4000 off\n", i);
            UARTprintf("led1 angle: %d\n", i);
            SysCtlDelay(25 * (SysCtlClockGet() / 3 / 1000));
        }

        UARTprintf("Pause B.\n");
        SysCtlDelay(1000 * (SysCtlClockGet() / 3 / 1000));
    }




//    while (1) {
//        for (int i = 500; i < 3500; i += 100) {
//            writeLed(0, i, 4000);
//            writeAngle(1, i / 10);
//            UARTprintf("led0 on: %d to 4000 off\n", i);
//            UARTprintf("led1 off: %d to 0 on\n", i);
//            SysCtlDelay(5 * (SysCtlClockGet() / 3 / 1000));
//        }
//
//        UARTprintf("Pause A.\n");
//        SysCtlDelay(1000 * (SysCtlClockGet() / 3 / 1000));
//
//        for (int i = 3500; i > 500; i -= 100) {
//            writeLed(0, i, 4000);
//            writeAngle(1, i / 10);
//            UARTprintf("led0 on: %d to 4000 off\n", i);
//            UARTprintf("led1 off: %d to 0 on\n", i);
//            SysCtlDelay(5 * (SysCtlClockGet() / 3 / 1000));
//        }
//
//        UARTprintf("Pause B.\n");
//        SysCtlDelay(1000 * (SysCtlClockGet() / 3 / 1000));
//    }

//    extern uint32_t i2c1_read(uint32_t slave_addr, uint8_t reg_addr, uint8_t *array, uint8_t cnt);
//    extern uint32_t i2c1_write(uint32_t slave_addr, uint8_t array[], uint8_t cnt);

//    for (uint8_t i = 0x00; i < 0x42; i++) {
//        BNOI2C1Receive(0x40, i, tmp, 1);
//        UARTprintf("Reg: 0x%02X = 0b%08B = 0x%02X\n", i, tmp, tmp);
//    }



    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    //BNO055_iERROR = bno055_test_read();
    //BNO055_iERROR = bno055_data_readout_template();


    if (BNO055_iERROR) {
        UARTprintf("BNO055 Init Error!\n");
    }
    else {
        UARTprintf("BNO055 Init Complete\n");
    }




    g_pI2CQueue = xQueueCreate(I2C_QUEUE_SIZE, I2C_ITEM_SIZE);

    if(xTaskCreate(I2CTask, (const portCHAR *)"I2C", I2CTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_I2C_TASK, NULL) != pdTRUE)
    {
        return 1;
    }
    UARTprintf("Task Init Complete\n");

    return 0;
}
