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

#include "xuartlite.h"
#include "xuartlite_l.h"

#include "xtmrctr.h"

#include "xintc.h"
#include "xil_exception.h"



#define LED_BASEADDR    XPAR_AXI_GPIO_0_BASEADDR
#define TIM_BASEADDR    XPAR_AXI_TIMER_0_BASEADDR
#define INT_BASEADDR    XPAR_MICROBLAZE_RISCV_0_AXI_INTC_BASEADDR

// 타이머 인터럽트 벡터
#define TIM_INT_VECTOR  XPAR_FABRIC_XTMRCTR_0_INTR

// 클럭
#define TIMER_CLK_HZ      100000000UL      // 100MHz
#define TOGG_PERIOD_MS  500UL            // 0.5초

// 카운트가 setValue  부터 0xFFFFFFFF까지 카운트 후 오버플로우
// 그래서 실제 리셋값은  = 0xFFFFFFFF - (클럭 * 주기) + 1
#define TIMER_TICKS     ((TIMER_CLK_HZ / 1000UL) *TOGG_PERIOD_MS)
#define TIMER_RESET_VAL (0xFFFFFFFF - TIMER_TICKS + 1UL)

// GPIO 채널
#define LED_CHANNEL 1


// 변수 선언
XGpio   led;            // GPIO_LED
XTmrCtr timer;          // 타이머 인스턴스
XIntc   intc;           // 인터럽트 콘트롤러 인스턴스

volatile u32 ledState = 0;      // LED 상태


// 타이머 ISR
void TimerISR(void *CallBackRef, u8 TimCtrNumber)
{
    // LED 상태를 반전
    ledState ^= 0xFFFF;

    // 출력
    XGpio_DiscreteWrite(&led, LED_CHANNEL, ledState);
}

// GPIO 초기화
int GPIO_Init()
{
    int status;
    status = XGpio_Initialize(&led, LED_BASEADDR);
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // 채널 1을(LED 16개) 전체를 출력으로 설정 (0)
    XGpio_SetDataDirection(&led, LED_CHANNEL, 0x00000000);

    // 초기에는 LED OFF
    XGpio_DiscreteWrite(&led, LED_CHANNEL, 0x00000000);

    return XST_SUCCESS;
}

// 타이머 초기화
int Timer_Init()
{
    int status;
        
    status = XTmrCtr_Initialize(&timer, TIM_BASEADDR);
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // 콜백 등록
    XTmrCtr_SetHandler(&timer, TimerISR, &timer);

    // 인터럽트 모드 + 자동 재시작
    XTmrCtr_SetOptions(&timer, 0, 
    XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION);

    // 타이머 리셋값 설정
    XTmrCtr_SetResetValue(&timer, 0, TIMER_RESET_VAL);

    return XST_SUCCESS;
}

// 인터럽트 콘틀로러 초기화
int Interrupt_Init()
{
    int status;

    status = XIntc_Initialize(&intc, INT_BASEADDR);
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // 타이머 ISR을 인터럽트 콘트롤러에 연결
    status = XIntc_Connect(&intc, TIM_INT_VECTOR, 
    (XInterruptHandler)XTmrCtr_InterruptHandler, &timer);
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // 인터럽트 콘트롤러 시작
    status = XIntc_Start(&intc, XIN_REAL_MODE);
    if(status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    // 타이머 인에이블
    XIntc_Enable(&intc, TIM_INT_VECTOR);

    // CPU레벨의 인터럽트 활성화
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, 
    (Xil_ExceptionHandler)XIntc_InterruptHandler, &intc);
    Xil_ExceptionEnable();

    return XST_SUCCESS;
}



int main()
{
    init_platform();

    // print("Hello World\n\r");
    // print("Successfully ran Hello World application");

    int status;
    
    xil_printf("Timer Interrupt LED Toggle START\r\n");

    // GPIO 초기화
    status = GPIO_Init();
    if(status != XST_SUCCESS)
    {
        xil_printf("GPIO INIT FAIL\r\n");
        return -1;
    }
    xil_printf("GPIO INIT SUCESS\r\n");

    //Timer Init
    status = Timer_Init();
    if(status != XST_SUCCESS)
    {
        xil_printf("TIMER INIT FAIL\r\n");
        return -1;
    }
    xil_printf("TIMER INIT SUCESS\r\n");

    // 인터럽트 초기화
    status = Interrupt_Init();
    if(status != XST_SUCCESS)
    {
        xil_printf("INT INIT FAIL\r\n");
        return -1;
    }
    xil_printf("INT INIT SUCESS\r\n");

    // 타이머 시작
    XTmrCtr_Start(&timer, 0);
    xil_printf("Timer START\r\n");

    while(1)
    {

    }

    cleanup_platform();
    return 0;
}