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
#include <xil_io.h>
#include <xil_types.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"

#include "sleep.h"

#include "xuartlite.h"
#include "xuartlite_l.h"



#define MyIP_FND_BASEADDR   XPAR_MYIP_FND_CONTR_0_BASEADDR

int main()
{
    init_platform();

    print("Hello World\n\r");
    print("Successfully ran Hello World application");
    

    u32 count = 0;

    while(1)
    {
        //reg_0에 직접 카운트 쓰기
        Xil_Out32(MyIP_FND_BASEADDR +0x00, count);
    }
    return 0;
}
