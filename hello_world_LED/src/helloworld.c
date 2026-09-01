
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
#include "platform.h"
#include "xil_printf.h"

#include "xparameters.h"
#include "sleep.h"
#include "xgpio.h"

#define GPIO_BASEADDR   XPAR_AXI_GPIO_0_BASEADDR
#define LED_CHANNEL     1
#define BTN_CHANNEL     2

#define BTN_UP          0x01        // bit 0
#define BTN_LEFT        0x02        // bit 1
#define BTN_RIGHT       0x04        // bit 2
#define BTN_DOWN        0x08        // bit 3




int main()
{
    init_platform();

    print("Hello World\n\r");
    print("Successfully ran Hello World application");


    XGpio gpio;

    //gpio init
    XGpio_Initialize(&gpio, XPAR_AXI_GPIO_0_BASEADDR);

    // 방향 설정
    // 0 => 출력, 1=> 입력
    // LED 설정
    XGpio_SetDataDirection(&gpio, LED_CHANNEL, 0x0000);

    // 버튼 설정
    XGpio_SetDataDirection(&gpio, BTN_CHANNEL, 0xFFFF);

    // 변수 선언
    int btnData = 0;
    int ledData = 0;

    while (1) 
    {
        // 버튼값을 읽어옴
        btnData = XGpio_DiscreteRead(&gpio, BTN_CHANNEL);

        if (btnData & BTN_UP)
        {
            ledData = 0x0001;
        }
        else if (btnData & BTN_LEFT)
        {
            ledData = 0x0002;
        }        
        else if (btnData & BTN_RIGHT)
        {
            ledData = 0x0004;            
        }
        else if (btnData & BTN_DOWN)
        {
            ledData = 0x0008;
        }
        else
        {
            ledData = 0x0000;   
        }      

        XGpio_DiscreteWrite(&gpio, LED_CHANNEL, ledData);

        msleep(10);

    }
    
    cleanup_platform();
    return 0;
}

