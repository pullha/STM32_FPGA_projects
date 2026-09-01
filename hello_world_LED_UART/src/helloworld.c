/******************************************************************************
* Copyright (C) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/
/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <sys/_intsup.h>
#include <xil_types.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xgpio.h"
#include "sleep.h"

#include "xuartlite.h"
#include "xuartlite_l.h"


#define GPIO_BASEADDR   XPAR_AXI_GPIO_0_BASEADDR
#define UART_BASEADDR   XPAR_AXI_UARTLITE_0_BASEADDR

#define LED_CHANNEL     1


XGpio       led;
XUartLite   uart;

// LED control function
void LED_AllOff()
{
    XGpio_DiscreteWrite(&led, LED_CHANNEL, 0x0000);
}

void LED_AllOn()
{
    XGpio_DiscreteWrite(&led, LED_CHANNEL, 0xFFFF);
}

void LED_Set(u16 value)
{
    XGpio_DiscreteWrite(&led, LED_CHANNEL, value);
}

void LED_ProcessCommand(u8 cmd)
{
    switch(cmd)
    {
        case 'a':
            LED_AllOff();
            printf(": LED ALL OFF\n\r");
            break;
        case 'b':
            LED_AllOn();
            xil_printf(": LED ALL ON\n\r");
            break;
        case '1':
            LED_Set(0x0001);
            print(": 1st LED ON\n\r");
            break;
        case '2':
            LED_Set(0x0002);
            print(": 2nd LED ON\n\r");
            break;
        case '3':
            LED_Set(0x0003);
            print(": 3rd LED ON\n\r");
            break;
        default:
            print(": ??\n\r");
            break;
    }
}


int main()
{
    init_platform();

    u8 rxData;
    int status;

    // GPIO 초기화
    status = XGpio_Initialize(&led, GPIO_BASEADDR);
    if(status != XST_SUCCESS) return XST_FAILURE;

    XGpio_SetDataDirection(&led, LED_CHANNEL, 0x0000);

    // uart 초기화
    status = XUartLite_Initialize(&uart, UART_BASEADDR);
    if(status != XST_SUCCESS) return XST_FAILURE;

    xil_printf("\r\n === UART LED CONTROL START ===\r\n");
    xil_printf("a : OFF, b : ON, 1~3 : LED Control\r\n");

    while(1)
    {
        // 데이터가 있는지 확인
        if(!XUartLite_IsReceiveEmpty(uart.RegBaseAddress))
        {
            // 1byte 수신
            XUartLite_Recv(&uart, &rxData, 1);

            // echo
            XUartLite_Send(&uart, &rxData, 1);

            LED_ProcessCommand(rxData);
        }
    }



    cleanup_platform();
    return 0;
}