// goBoard Bringup Development
// Author: Paul Schneider

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>

#include "inc/tm4c123fh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_can.h"

#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/can.h"

void delay_ms(int milliseconds) {
    SysCtlDelay(SysCtlClockGet() / 3 * milliseconds / 1000);
}
void delay_sec(int seconds) {
    SysCtlDelay(SysCtlClockGet() / 3 * seconds);
}



volatile uint32_t g_ui32TxCount = 0;
volatile uint32_t g_ui32RxCount = 0;
volatile bool g_bRXFlag = 0;
volatile bool g_bErrFlag = 0;

volatile uint32_t canStatusControlTXOK = 0;
volatile uint32_t canStatusControlERROR = 0;
volatile uint32_t canStatusControlANY = 0;

void CAN0IntHandler(void) {
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

//        CAN_STATUS_BUS_OFF - controller is in bus-off condition
//        CAN_STATUS_EWARN - an error counter has reached a limit of at least 96
//        CAN_STATUS_EPASS - CAN controller is in the error passive state
//        CAN_STATUS_RXOK - a message was received successfully (independent of any message filtering).
//        CAN_STATUS_TXOK - a message was successfully transmitted
//        CAN_STATUS_LEC_MSK - mask of last error code bits (3 bits)
//        CAN_STATUS_LEC_NONE - no error
//        CAN_STATUS_LEC_STUFF - stuffing error detected
//        CAN_STATUS_LEC_FORM - a format error occurred in the fixed format part of a message
//        CAN_STATUS_LEC_ACK - a transmitted message was not acknowledged
//        CAN_STATUS_LEC_BIT1 - dominant level detected when trying to send in recessive
//        mode
//        CAN_STATUS_LEC_BIT0 - recessive level detected when trying to send in dominant
//        mode
//        CAN_STATUS_LEC_CRC - CRC error in received message




        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    //
    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32TxCount++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}
void CAN1IntHandler(void) {
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ui32Status = CANStatusGet(CAN1_BASE, CAN_STS_CONTROL);

        if (ui32Status == CAN_STATUS_TXOK) {
            canStatusControlTXOK++;
        }
        else {
            canStatusControlERROR++;
        }

        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    //
    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN1_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32TxCount++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    else if(ui32Status == 2) {

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // received.  In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        g_ui32RxCount++;

        //
        // Set flag to indicate received message is pending.
        //
        g_bRXFlag = 1;

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag = 0;
    }







    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}
void InitCAN() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);
    //CANIntRegister(CAN0_BASE, CANIntHandler); // use dynamic vector table allocation
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);
    CANEnable(CAN0_BASE);
}

int main(void)
{
    //puts("Hello!");
    printf("goBoard Boot!\n");
    //printf("Boot SysClock = %d\n", SysCtlClockGet());

    // Sys clock 200MHz PLL --> 200/8 = 25MHz
    SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ | SYSCTL_SYSDIV_8 | SYSCTL_USE_PLL);

    //printf("Mod SysClock = %d\n", SysCtlClockGet());

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);


    tCANMsgObject sCANmsgRx;
    uint64_t ui64RxData;
    uint8_t *pui8RxData;

    tCANMsgObject sCANmsgTx;
    uint32_t ui32TxData;
    uint8_t *pui8TxData;

    pui8TxData = (uint8_t *)&ui32TxData;
    pui8RxData = (uint8_t *)&ui64RxData;

    //

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_CAN1RX);
    GPIOPinConfigure(GPIO_PA1_CAN1TX);

    GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);

    CANInit(CAN1_BASE);

    CANBitRateSet(CAN1_BASE, SysCtlClockGet(), 500000);


    // CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
    // See ..._startup_ccs.c for interrupt mapping
    CANIntEnable(CAN1_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN1);

    CANEnable(CAN1_BASE);

    ui32TxData = 0x0ABCDEF0;
    sCANmsgTx.ui32MsgID = 1;
    sCANmsgTx.ui32MsgIDMask = 0;
    sCANmsgTx.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANmsgTx.ui32MsgLen = sizeof(pui8TxData);
    sCANmsgTx.pui8MsgData = pui8TxData;

    ui64RxData = 0;
    sCANmsgRx.ui32MsgID = 0;
    sCANmsgRx.ui32MsgIDMask = 0;
    sCANmsgRx.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sCANmsgRx.ui32MsgLen = 8;

    CANMessageSet(CAN1_BASE, 2, &sCANmsgRx, MSG_OBJ_TYPE_RX);

    unsigned int uIdx;

    printf("start loop\n");
    while (1) {
        //printf("tick\n");

// ---- Rx ----
        if(g_bRXFlag) {
            //
            // Reuse the same message object that was used earlier to configure
            // the CAN for receiving messages.  A buffer for storing the
            // received data must also be provided, so set the buffer pointer
            // within the message object.
            //
            sCANmsgRx.pui8MsgData = pui8RxData;

            //
            // Read the message from the CAN.  Message object number 1 is used
            // (which is not the same thing as CAN ID).  The interrupt clearing
            // flag is not set because this interrupt was already cleared in
            // the interrupt handler.
            //
            CANMessageGet(CAN0_BASE, 2, &sCANmsgRx, 0);

            //
            // Clear the pending message flag so that the interrupt handler can
            // set it again when the next message arrives.
            //
            g_bRXFlag = 0;

            //
            // Check to see if there is an indication that some messages were
            // lost.
            //
            if(sCANmsgRx.ui32Flags & MSG_OBJ_DATA_LOST)
            {
                printf("CAN message loss detected\n");
            }

            //
            // Print out the contents of the message that was received.
            //
            printf("Msg ID=0x%08X len=%u data=0x",
                       sCANmsgRx.ui32MsgID, sCANmsgRx.ui32MsgLen);
            for(uIdx = 0; uIdx < sCANmsgRx.ui32MsgLen; uIdx++)
            {
                printf("%02X ", pui8RxData[uIdx]);
            }
            printf("total count=%u\n", g_ui32TxCount);
        }

// ---- Tx ----
//        printf("Sending msg: 0x%02X %02X %02X %02X\n",
//                   pui8TxData[0], pui8TxData[1], pui8TxData[2],
//                   pui8TxData[3]);

        CANMessageSet(CAN1_BASE, 1, &sCANmsgTx, MSG_OBJ_TYPE_TX);

        delay_ms(100);

        if(g_bErrFlag) printf(" error - cable connected?\n");
        //else printf(" total count = %u\n", g_ui32TxCount);

        if (!((ui32TxData - 1) % 20)) {
            printf("tx: %d   txok: %d   err: %d\n", g_ui32TxCount, canStatusControlTXOK, canStatusControlERROR);
        }

        ui32TxData++;

    } // End while


} // End main()
