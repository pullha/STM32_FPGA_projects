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
#include "platform.h"
#include "xil_printf.h"

#include "xparameters.h"
#include "xgpio.h"
#include "sleep.h"

#include "xuartlite.h"
#include "xintc.h"

#include "xil_exception.h"
#include "xstatus.h"

#define UART_BASEADDR   XPAR_AXI_UARTLITE_0_BASEADDR
#define INTC_BASEADDR   XPAR_MICROBLAZE_RISCV_0_AXI_INTC_BASEADDR
#define UART_INT_IRQ    XPAR_FABRIC_AXI_UARTLITE_0_INTR

XUartLite   uart;
XIntc       InterruptController;

static u8 rxBuffer[1];


// 수신 콜백
void receHandler(void *CallBackRef, unsigned int data)
{
    XUartLite *uartPtr = (XUartLite *)CallBackRef;

    // data <<<< 실제 수신된 데이터    
    if(data > 0)
    {
        // 받은 데이터를 그대로 전송
        XUartLite_Send(uartPtr, rxBuffer, data);
        
        // 다음 수신 등록을위해...(이거 안하면 1번만 동작함)
        XUartLite_Recv(uartPtr, rxBuffer, 1);
    }
}

void sendHandler(void *CallBackRef, unsigned int data)
{

}


// uart 초기화
static int initUart()
{
    int status;
    XUartLite_Config *confPtr;

    // 설정 구조체 
    confPtr = XUartLite_LookupConfig(UART_BASEADDR);
    if(confPtr == NULL)
    {
        return XST_FAILURE;
    }    

    // 드라이버 초기화
    status = XUartLite_CfgInitialize(&uart, confPtr, confPtr->RegBaseAddr);
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // check (하드웨어가 정상인지?)
    status = XUartLite_SelfTest(&uart);
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

// 인터럽트 콘트롤러 초기화
static int initIntc()
{
    int status;

    // 콘트롤러 초기화
    status = XIntc_Initialize(&InterruptController, UART_INT_IRQ);  // 요기 ... IRQ 번호 넘겨야 함 !!
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

// 인터럽트 설정
static int setupInterrupt()
{
    int status;

    // uart 인터럽트를 xintc 에 연결
    status = XIntc_Connect(&InterruptController, UART_INT_IRQ, 
    (XInterruptHandler)XUartLite_InterruptHandler, (void *)&uart);
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // 콜백 등록
    // uart에 데이터 들어오면 receHandler 실행 -> &Uartlite 넘기시오
    XUartLite_SetRecvHandler(&uart, receHandler, (void *)&uart);
    // 요기 수정 ... 센드 핸들러 있어야 함 !!!(추가함)
    XUartLite_SetSendHandler(&uart, sendHandler, (void *)&uart);

    // 인터럽트 콘트롤러 시작
    status = XIntc_Start(&InterruptController, XIN_REAL_MODE);
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // 특정 인터럽트 인에이블
    XIntc_Enable(&InterruptController, UART_INT_IRQ);

    // uart 내부 인터럽트 인에이블
    XUartLite_EnableInterrupt(&uart);

    // CPU에 인터럽트 연결
    Xil_ExceptionInit();

    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
     (Xil_ExceptionHandler)XIntc_InterruptHandler,(void *)&InterruptController);

    Xil_ExceptionEnable();

    return XST_SUCCESS;
}


int main()
{
    init_platform();

    print("Hello World\n\r");
    print("Successfully ran Hello World application");

    int status;

    // uart 초기화
    status = initUart();
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // 인터럽트 콘트롤러 초기화
    status = initIntc();
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // 인터럽트 설정
    status = setupInterrupt();
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // 수신등록
    XUartLite_Recv(&uart, rxBuffer, 1);

    while(1)
    {

    }

    cleanup_platform();
    return 0;
}