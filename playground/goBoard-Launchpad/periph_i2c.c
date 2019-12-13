
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

#include "periph_i2c.h"

extern void UARTprintf(const char *pcString, ...);

uint8_t readI2C(uint32_t ui32base, uint8_t slave_addr, uint8_t reg_addr, uint8_t *array, uint8_t cnt)
{
    // If single count expected, follow generic protocol
    if (cnt == 1) {
        //specify that we are writing (a register address) to the
        //slave device
        I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);

        //specify register to be read
        I2CMasterDataPut(I2C1_BASE, reg_addr);

        //send control byte and register address byte to slave device
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        //wait for MCU to finish transaction
        while(I2CMasterBusy(I2C1_BASE));

        //specify that we are going to read from slave device
        I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, true);

        //send control byte and read from the register we
        //specified
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

        //wait for MCU to finish transaction
        while(I2CMasterBusy(I2C1_BASE));

        //return data pulled from the specified register
        array[0] = I2CMasterDataGet(I2C1_BASE);

        return 0;
    }
    else {
        // cycle through


        for (uint8_t i = 0; i < cnt; i++) {

            I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);

            //specify register to be read
            I2CMasterDataPut(I2C1_BASE, reg_addr + i);

            //send control byte and register address byte to slave device
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

            //wait for MCU to finish transaction
            while(I2CMasterBusy(I2C1_BASE));

            //specify that we are going to read from slave device
            I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, true);

            //send control byte and read from the register we
            //specified
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

            //wait for MCU to finish transaction
            while(I2CMasterBusy(I2C1_BASE));

            //return data pulled from the specified register
            array[i] = I2CMasterDataGet(I2C1_BASE);
        }

        return 0;

    }

    // TODO ADD FAULT CHECKS
}

uint8_t writeI2C(uint32_t ui32base, uint8_t slave_addr, uint8_t array[], uint8_t cnt) {

    // Set I2C slave addr for TX
    I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);

    // Register address into MDR
    I2CMasterDataPut(I2C1_BASE, array[0]);

    // Initialize TX transaction, wait to finish
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C1_BASE));

    for (uint8_t i = 1; i < cnt - 1; i++) {
        // Register Data into MDR
        I2CMasterDataPut(I2C1_BASE, array[i]);

        // Initialize TX transaction, wait to finish
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        while(I2CMasterBusy(I2C1_BASE));

    }

    I2CMasterDataPut(I2C1_BASE, array[cnt - 1]);

    // Initialize TX transaction, wait to finish
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C1_BASE));


    return 0;
}

uint8_t initI2C(uint32_t ui32base) {
    UARTprintf("\nInitializing I2C1\n");

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
    I2CMasterTimeoutSet(I2C1_BASE, 0x0F); // TODO clock stretching

    UARTprintf("I2C1 Init Complete\n");
    return 0; //TODO
}
