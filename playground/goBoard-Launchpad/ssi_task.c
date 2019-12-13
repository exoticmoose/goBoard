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
#include "driverlib/ssi.h"
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
#include "drivers/buttons.h"
#include "utils/uartstdio.h"

#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "nRF24/RF24.h"
#include "nRF24/RF24_config.h"
#include "nRF24/nRF24L01.h"

const uint64_t pipes[2] = { 0xF0F0F0F0F0LL, 0xF0F0F0F0D2LL };

RF24_data g_RF24_data =
{
    SYSCTL_PERIPH_GPIOB,    // 0xF0000800
    GPIO_PORTB_BASE,        // 0x40004000
    GPIO_PIN_2,             // 0x00000040
    SYSCTL_PERIPH_GPIOB,    // 0xF0000800
    GPIO_PORTB_BASE,        // 0x40004000
    GPIO_PIN_3,             // 0x00000080
    true,       // RF24_WB
    false,      // RF24_PV
    32,         // Payload Size (0..32)
    false,      // Ack Payload Available
    false,      // Dynamic Payloads Enabled
    0           // Pipe0 Reading Address
};


#define SSITASKSTACKSIZE        128         // Stack size in words

// Message queue size for SSI transaction (read/write) requests
#define SSI_ITEM_SIZE           sizeof(uint8_t)
#define SSI_QUEUE_SIZE          32



extern xQueueHandle g_pI2CQueue;
extern xQueueHandle g_pGPIOQueue;
xQueueHandle g_pSSIQueue;


static void SSITask(void *pvParameters) {
    UARTprintf("\nSSI First Tick!\n");
    portTickType ui32WakeTime = xTaskGetTickCount();
    RF24_startListening();

    uint8_t rx[5];


    while (1) {
        //UARTprintf("\nSSI tick!\n");
        if (RF24_available()) {
            RF24_read(rx, sizeof(rx));
        }

        UARTprintf("%d %d %d %d %d %d\n", rx[0], rx[1], rx[2], rx[3], rx[4]);

        if(xQueueSend(g_pI2CQueue, &rx, portMAX_DELAY) !=
           pdPASS)
        {
            //
            // Error. The queue should never be full. If so print the
            // error message on UART and wait for ever.
            //
            UARTprintf("\nQueue full. This should never happen.\n");
            while(1)
            {
            }
        }
        if(xQueueSend(g_pGPIOQueue, &rx, portMAX_DELAY) !=
           pdPASS)
        {
            //
            // Error. The queue should never be full. If so print the
            // error message on UART and wait for ever.
            //
            UARTprintf("\nQueue full. This should never happen.\n");
            while(1)
            {
            }
        }

//        uint8_t packet[10] = "Why hello";
//        UARTprintf("Now sending %u...", packet);
//
//        bool ok = RF24_write( &packet, sizeof(packet) );
//
//        if (ok)
//          UARTprintf("ok...");
//        else
//          UARTprintf("failed.\n\r");

        // Now, continue listening

//
//        // Wait here until we get a response, or timeout (250ms)
//        uint32_t started_waiting_at = millis();
//        bool timeout = false;
//        while ( ! RF24_available() && ! timeout )
//          if (millis() - started_waiting_at > 200 )
//            timeout = true;
//
//        // Describe the results
//        if ( timeout )
//        {
//          UARTprintf("Failed, response timed out.\n\r");
//        }
//        else
//        {
//          // Grab the response, compare, and send to debugging spew
//            uint32_t got_time;
//          RF24_read( &got_time, sizeof(uint32_t) );
//
//          // Spew it
//          UARTprintf("Got response %u, round-trip delay: %u\n\r",got_time,millis()-got_time);
//        }
        vTaskDelay(50);
    }


}



int SSITaskInit() {

    // SSI initialization


    uint32_t ui32DataRx;
    // Enable peripheral for SSI/SPI.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);

    // Configure the muxing and GPIO settings to bring the SSI functions out to the pins
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinConfigure(GPIO_PB5_SSI2FSS);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);

    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4);

    // Configure and enable the SSI port.  Use SSI2, system clock, master mode, 1MHz SSI frequency, and 8-bit data
    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);

    // Enable the SSI2 module.
    SSIEnable(SSI2_BASE);



     // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the TI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    while(SSIDataGetNonBlocking(SSI2_BASE, &ui32DataRx))
    {
    }

    RF24_begin();
    RF24_setRetries( 15, 3 );

    RF24_openWritingPipe(pipes[1]);
    RF24_openReadingPipe(1,pipes[0]);

    RF24_startListening();

    //
    // Dump the configuration of the rf unit for debugging
    //
    RF24_printDetails();

    RF24_setDataRate( RF24_250KBPS );
    RF24_setPALevel( RF24_PA_MIN ) ;
    //tmp = RF24_getDataRate();

    g_pSSIQueue = xQueueCreate(SSI_QUEUE_SIZE, SSI_ITEM_SIZE);

    if(xTaskCreate(SSITask, (const portCHAR *)"SSI", SSITASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SSI_TASK, NULL) != pdTRUE)
    {
        return 1;
    }
    UARTprintf("SSI Task Init Complete\n");

    return 0;
}
