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
#include <xil_types.h>
#include "platform.h"
#include "xil_printf.h"

#include "xparameters.h"
#include "xgpio.h"
#include "sleep.h"

#define FND_BASEADDR        XPAR_AXI_GPIO_0_BASEADDR
#define LED_BASEADDR        XPAR_AXI_GPIO_1_BASEADDR

#define FND_AN_CHANNEL      1
#define FND_DISP_CHANNEL    2
#define LED_CHANNEL         1




int main()
{
    init_platform();

    print("Hello World\n\r");
    print("Successfully ran Hello World application");

    XGpio fnd;

    u32 count = 0;
    u32 loopCount = 0;

    // font (Active High)
    u8 fnd_font[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};

    // 자릿수 선택 

    // 초기화
    XGpio_Initialize(&fnd, FND_BASEADDR);

    // 방향 설정
    XGpio_SetDataDirection(&fnd, FND_AN_CHANNEL, 0x0);      // 출력
    XGpio_SetDataDirection(&fnd, FND_DISP_CHANNEL, 0x0);    // 출력


    while (1) 
    {

        // 4개의 자리를 분리
        u8 digit[4];
        digit[0] = count % 10;              // 1의 자리
        digit[1] = (count/ 10) % 10 ;       // 10의 자리
        digit[2] = (count/ 100) % 10 ;      // 100의 자리
        digit[3] = (count/ 1000) % 10 ;     // 1000의 자리

        // 4개의 자리를 scan
        for(int i = 0; i<4 ; i++)
        {
            XGpio_DiscreteWrite(&fnd, FND_AN_CHANNEL, anode[i]);
            XGpio_DiscreteWrite(&fnd, FND_DISP_CHANNEL, ~fnd_font[digit[i]]);
            msleep(1); // 각 자리당 1ms
        }

        // 카운트 증가 속도
        loopCount ++;
        if(loopCount > 50)
        {
            count++;
            if(count > 9999) count = 0;
            loopCount = 0;
        }



        // // 자릿수 선택(Active Low -> On)
        // // 마지막 자릿수 : 0b1110 == 0xE
        // XGpio_DiscreteWrite(&fnd, FND_AN_CHANNEL, 0xE);

        // // 숫자를 표시
        // XGpio_DiscreteWrite(&fnd, FND_DISP_CHANNEL, ~fnd_font[count % 10]);

        // // 카운트 증가
        // loopCount++;
        // if(loopCount > 100)
        // {
        //     count++;
        //     loopCount = 0;
        // }
        // msleep(1);
    }


    cleanup_platform();
    return 0;
}
