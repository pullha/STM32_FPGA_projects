/******************************************************************************
 * AXI Timer PWM을 이용한 LED Fade 예제
 *
 * 동작:
 *  1. AXI Timer의 PWM 전용 API를 사용한다.
 *  2. Timer 0은 PWM 전체 주기를 담당한다.
 *  3. Timer 1은 PWM HIGH 시간을 담당한다.
 *  4. 듀티비를 0%에서 100%까지 변경한다.
 *  5. LED가 서서히 밝아졌다가 어두워진다.
 *
 * PWM 주파수 : 1 kHz
 * PWM 주기   : 1 ms
 * Fade 시간  : 약 1초 증가 + 약 1초 감소
 *
 * 주의:
 *  - AXI Timer PWM 기능은 Timer 0과 Timer 1을 모두 사용한다.
 *  - XTmrCtr_PwmConfigure()의 시간 단위는 ns(나노초)이다.
 *  - MicroBlaze V의 SDT 환경에서는 Device ID 대신 Base Address를
 *    사용하여 XTmrCtr_Initialize()를 호출한다.
 ******************************************************************************/

#include <stdio.h>
#include <xil_types.h>

#include "platform.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "sleep.h"

#include "xtmrctr.h"

/*==========================================================================*/
/* 하드웨어 설정                                                             */
/*==========================================================================*/

/*
 * AXI Timer Base Address
 *
 * MicroBlaze V + SDT 방식에서는 다음과 같은 DEVICE_ID 매크로가
 * 생성되지 않을 수 있다.
 *
 *   XPAR_AXI_TIMER_0_DEVICE_ID
 *
 * 따라서 xparameters.h에 생성되어 있는 BASEADDR를 사용한다.
 */
#define TIMER_BASEADDR          XPAR_AXI_TIMER_0_BASEADDR

/*
 * AXI Timer 입력 클럭
 *
 * Vivado Block Design에서 AXI Timer의 s_axi_aclk가
 * 100 MHz로 연결되어 있다고 가정한다.
 */
#define TIMER_CLK_HZ            100000000UL

/*
 * PWM 주파수
 *
 * 1 kHz = 1초 동안 1,000번 반복
 */
#define PWM_FREQ_HZ             1000UL

/*
 * 1초 = 1,000,000,000 ns
 *
 * PWM 주기:
 *
 *   1,000,000,000 ns / 1,000 Hz
 *       = 1,000,000 ns
 *       = 1 ms
 */
#define PWM_PERIOD_NS           (1000000000UL / PWM_FREQ_HZ)

/*
 * AXI Timer 한 클럭의 시간
 *
 * 100 MHz인 경우:
 *
 *   1,000,000,000 ns / 100,000,000 Hz
 *       = 10 ns
 */
#define TIMER_CLK_PERIOD_NS     (1000000000UL / TIMER_CLK_HZ)

/*
 * XTmrCtr_PwmConfigure()가 허용하는 최소 HIGH 시간은
 * AXI Timer 클럭 두 주기 이상이다.
 *
 * 100 MHz인 경우:
 *
 *   10 ns × 2 = 20 ns
 *
 * 따라서 duty = 0%일 때 완전히 0 ns를 설정하는 대신
 * 최소 HIGH 시간인 20 ns를 설정한다.
 *
 * LED에서는 사실상 꺼진 것처럼 보인다.
 */
#define PWM_MIN_HIGH_NS         (2UL * TIMER_CLK_PERIOD_NS)

/*
 * 각 듀티 단계의 대기시간
 *
 * 10 ms × 101단계 ≒ 1.01초
 */
#define FADE_STEP_DELAY_US      10000U

/* AXI Timer 드라이버 인스턴스 */
static XTmrCtr Timer;


/*==========================================================================*/
/* PWM 듀티비 설정 함수                                                      */
/*==========================================================================*/

/**
 * PWM_SetDuty()
 *
 * 역할:
 *   PWM 주파수는 유지하면서 듀티비만 변경한다.
 *
 * 매개변수:
 *   duty : 설정할 듀티비, 0~100%
 *
 * 반환값:
 *   AXI Timer 드라이버가 계산한 실제 듀티비
 *
 * AXI Timer PWM 구조:
 *
 *   Timer 0 : PWM 전체 주기
 *   Timer 1 : PWM HIGH 시간
 *
 * 주의:
 *   XTmrCtr_PwmConfigure()의 두 번째와 세 번째 인수는
 *   Timer 클럭 수가 아니라 ns 단위의 시간이다.
 */
static u8 PWM_SetDuty(int duty)
{
    u32 highTimeNs;
    u8 actualDuty;

    /*
     * 전달된 듀티비를 0~100% 범위로 제한한다.
     */
    if (duty < 0)
    {
        duty = 0;
    }
    else if (duty > 100)
    {
        duty = 100;
    }

    /*
     * 듀티비에 따른 PWM HIGH 시간을 계산한다.
     *
     * 계산식:
     *
     *   HIGH 시간 = PWM 전체 주기 × duty / 100
     *
     * 예:
     *
     *   PWM 주기 = 1,000,000 ns
     *   duty     = 30%
     *
     *   HIGH 시간
     *     = 1,000,000 × 30 / 100
     *     = 300,000 ns
     *     = 0.3 ms
     */

    if (duty == 0)
    {
        /*
         * XTmrCtr_PwmConfigure()는 HIGH 시간이 Timer 클럭
         * 두 주기보다 작으면 XST_INVALID_PARAM을 반환한다.
         *
         * 따라서 0%에서는 최소 허용 시간인 20 ns를 설정한다.
         *
         * 1 ms 중 20 ns만 HIGH이므로 실제 듀티비는:
         *
         *   20 ns / 1,000,000 ns × 100
         *     = 0.002%
         *
         * LED에서는 사실상 꺼진 상태로 보인다.
         */
        highTimeNs = PWM_MIN_HIGH_NS;
    }
    else if (duty == 100)
    {
        /*
         * 100%에서는 HIGH 시간을 PWM 전체 주기와 같게 한다.
         */
        highTimeNs = PWM_PERIOD_NS;
    }
    else
    {
        /*
         * 중간 듀티비의 HIGH 시간을 계산한다.
         *
         * u64로 먼저 계산하여 곱셈 중 발생할 수 있는
         * 정수 오버플로우를 방지한다.
         */
        highTimeNs =
            (u32)(((u64)PWM_PERIOD_NS * (u64)duty) / 100ULL);
    }

    /*
     * PWM이 실행 중인 상태에서 설정값을 변경하지 않도록
     * 먼저 PWM을 비활성화한다.
     *
     * XTmrCtr_PwmDisable()은:
     *
     *   1. Timer 0 정지
     *   2. Timer 1 정지
     *   3. 두 타이머의 카운터 Reset
     *   4. PWM 출력 비활성화
     *
     * 작업을 수행한다.
     */
    XTmrCtr_PwmDisable(&Timer);

    /*
     * 새로운 PWM 주기와 HIGH 시간을 설정한다.
     *
     * 내부적으로 다음 작업이 수행된다.
     *
     *   - Timer 0과 Timer 1 정지
     *   - Down Count 모드 설정
     *   - Auto Reload 모드 설정
     *   - Timer 0에 PWM 주기 설정
     *   - Timer 1에 PWM HIGH 시간 설정
     *
     * 따라서 사용자가 직접 다음 함수를 호출할 필요가 없다.
     *
     *   XTmrCtr_SetOptions()
     *   XTmrCtr_SetResetValue()
     */
    actualDuty = XTmrCtr_PwmConfigure(
        &Timer,
        PWM_PERIOD_NS,
        highTimeNs
    );

    /*
     * 설정된 PWM 출력을 활성화한다.
     *
     * XTmrCtr_PwmEnable()은 Timer 0과 Timer 1을 시작하고
     * PWM 출력을 활성화한다.
     */
    XTmrCtr_PwmEnable(&Timer);

    return actualDuty;
}


/*==========================================================================*/
/* main 함수                                                                 */
/*==========================================================================*/

int main(void)
{
    int status;
    int duty;
    u8 actualDuty;

    /*
     * BSP에서 설정한 UART 및 플랫폼 초기화
     */
    init_platform();

    xil_printf("\r\n");
    xil_printf("====================================\r\n");
    xil_printf(" AXI Timer PWM LED Fade Start\r\n");
    xil_printf("====================================\r\n");

    /*
     * AXI Timer 드라이버 초기화
     *
     * MicroBlaze V의 SDT 방식에서는 두 번째 인수로
     * Device ID가 아니라 Base Address를 전달한다.
     */
    status = XTmrCtr_Initialize(
        &Timer,
        TIMER_BASEADDR
    );

    if (status != XST_SUCCESS)
    {
        xil_printf("Timer Init Failed\r\n");
        cleanup_platform();
        return XST_FAILURE;
    }

    xil_printf("Timer Init Success\r\n");

    /*
     * PWM 설정 정보 출력
     *
     * xil_printf()에서는 %lu 지원이 제한될 수 있으므로
     * 32비트 정수값으로 출력한다.
     */
    xil_printf("Timer Clock  : %d Hz\r\n",
               (int)TIMER_CLK_HZ);

    xil_printf("PWM Frequency: %d Hz\r\n",
               (int)PWM_FREQ_HZ);

    xil_printf("PWM Period   : %d ns\r\n",
               (int)PWM_PERIOD_NS);

    xil_printf("Min High Time: %d ns\r\n",
               (int)PWM_MIN_HIGH_NS);

    /*
     * 초기 듀티비를 사실상 0%로 설정한다.
     *
     * PWM_SetDuty() 내부에서 다음 순서로 실행된다.
     *
     *   PwmDisable
     *       ↓
     *   PwmConfigure
     *       ↓
     *   PwmEnable
     */
    actualDuty = PWM_SetDuty(0);

    xil_printf("Initial Duty : %d%%\r\n",
               (int)actualDuty);

    xil_printf("PWM Running...\r\n");


    /*======================================================================*/
    /* LED Fade 반복                                                         */
    /*======================================================================*/

    while (1)
    {
        /*
         * LED 밝기 증가
         *
         * 듀티비:
         *
         *   0%, 1%, 2%, ... 98%, 99%, 100%
         *
         * 각 단계마다 10 ms 대기한다.
         *
         * 전체 증가 시간:
         *
         *   101단계 × 10 ms ≒ 1.01초
         */
        for (duty = 0; duty <= 100; duty++)
        {
            PWM_SetDuty(duty);

            usleep(FADE_STEP_DELAY_US);
        }

        /*
         * 증가 루프의 마지막이 100%이고 감소 루프의 처음도
         * 100%이면 100%가 두 번 실행된다.
         *
         * 이를 피하기 위해 감소는 99%부터 시작한다.
         *
         * 듀티비:
         *
         *   99%, 98%, 97%, ... 2%, 1%, 0%
         */
        for (duty = 99; duty >= 0; duty--)
        {
            PWM_SetDuty(duty);

            usleep(FADE_STEP_DELAY_US);
        }
    }

    /*
     * 무한 반복문으로 인해 정상 동작 중에는
     * 아래 코드에 도달하지 않는다.
     */
    cleanup_platform();

    return XST_SUCCESS;
}