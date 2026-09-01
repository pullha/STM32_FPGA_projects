/******************************************************************************
 * AXI Smart Sorter - OV7670 no-FIFO camera signal/status test
 * Board : Zybo Z7-20
 * Tool  : Vitis 2024.2
 ******************************************************************************/

#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"
#include "xil_types.h"

#define CAMERA_BASE XPAR_CAMERA_CONTROLLER_0_BASEADDR

#define CAMERA_CONTROL_OFFSET 0x00U
#define CAMERA_INPUT_OFFSET   0x04U
#define CAMERA_STATUS_OFFSET  0x08U
#define CAMERA_COLOR_OFFSET   0x0CU

#define SERVO_BASE XPAR_SERVO_CONTROLLER_0_BASEADDR
#define SERVO_PWM_OFFSET 0x00U

/* Existing camera_control_reg bit mapping. */
#define CAM_SIOC     (1U << 4)
#define CAM_SIOD_OUT (1U << 5)
#define CAM_SIOD_OE  (1U << 6)
#define CAM_RST      (1U << 8)

#define CAM_VSYNC_BIT 8U
#define CAM_HREF_BIT  9U
#define CAM_SIOD_BIT  10U
#define CAM_PCLK_BIT  11U

#define FAST_POLL_COUNT 5000000U
#define MAX_TRANSITIONS 16U
#define OV7670_SCCB_ADDR 0x42U

#define SERVO_SKY_US    1100U
#define SERVO_BLOCK_US  1500U
#define SERVO_ORANGE_US 1900U
#define SERVO_PERIOD_US 20000U
#define DECISION_SAMPLES 10
#define COLOR_MIN_COUNT 5U

struct ov7670_reg_value {
    u8 reg;
    u8 value;
};

static const struct ov7670_reg_value ov7670_rgb565_qvga_init[] = {
    {0x12, 0x80}, /* COM7 reset */
    {0xFF, 0xFF},
    {0x11, 0x01}, /* CLKRC: input clock / 2 */
    {0x12, 0x14}, /* COM7: QVGA + RGB */
    {0x40, 0xD0}, /* COM15: RGB565, full range */
    {0x8C, 0x00}, /* RGB444 disable */
    {0x3A, 0x04}, /* TSLB: RGB byte order */
    {0x3D, 0x88}, /* COM13: gamma/UV settings, RGB order bit */
    {0x0C, 0x04}, /* COM3: enable scaling */
    {0x3E, 0x19}, /* COM14: manual scaling, PCLK divider */
    {0x70, 0x3A},
    {0x71, 0x35},
    {0x72, 0x11},
    {0x73, 0xF1},
    {0xA2, 0x02},
    {0x15, 0x00}, /* COM10: normal VSYNC/HREF/PCLK polarity */
    {0x17, 0x16},
    {0x18, 0x04},
    {0x32, 0x80},
    {0x19, 0x02},
    {0x1A, 0x7A},
    {0x03, 0x0A},
    {0x0E, 0x61},
    {0x0F, 0x4B},
    {0x16, 0x02},
    {0x1E, 0x07},
    {0x21, 0x02},
    {0x22, 0x91},
    {0x29, 0x07},
    {0x33, 0x0B},
    {0x35, 0x0B},
    {0x37, 0x1D},
    {0x38, 0x71},
    {0x39, 0x2A},
    {0x3C, 0x78},
    {0x4D, 0x40},
    {0x4E, 0x20},
    {0x69, 0x00},
    {0x6B, 0x0A},
    {0x74, 0x10},
    {0x8D, 0x4F},
    {0x8E, 0x00},
    {0x8F, 0x00},
    {0x90, 0x00},
    {0x91, 0x00},
    {0x96, 0x00},
    {0x9A, 0x80},
    {0xB0, 0x84},
    {0xB1, 0x0C},
    {0xB2, 0x0E},
    {0xB3, 0x82},
    {0xB8, 0x0A},
    {0xFF, 0x00}
};

static u32 camera_control_shadow;

static void camera_write_control(u32 value)
{
    camera_control_shadow = value;
    Xil_Out32(CAMERA_BASE + CAMERA_CONTROL_OFFSET, camera_control_shadow);
}

static u32 camera_read_input(void)
{
    return Xil_In32(CAMERA_BASE + CAMERA_INPUT_OFFSET);
}

static u32 camera_read_status(void)
{
    return Xil_In32(CAMERA_BASE + CAMERA_STATUS_OFFSET);
}

static u32 camera_read_color(void)
{
    return Xil_In32(CAMERA_BASE + CAMERA_COLOR_OFFSET);
}

static void servo_write(int high)
{
    Xil_Out32(SERVO_BASE + SERVO_PWM_OFFSET, high ? 1U : 0U);
}

static int bit_from(u32 value, u32 bit)
{
    return (int)((value >> bit) & 0x01U);
}

static void sccb_delay(void)
{
    usleep(5);
}

static void sccb_set(int sioc, int siod_drive_low)
{
    u32 value = CAM_RST;

    if (sioc != 0) {
        value |= CAM_SIOC;
    }
    if (siod_drive_low != 0) {
        value |= CAM_SIOD_OE;
    }

    camera_write_control(value);
    sccb_delay();
}

static int sccb_read_siod(void)
{
    return bit_from(camera_read_input(), CAM_SIOD_BIT);
}

static void sccb_start(void)
{
    sccb_set(1, 0);
    sccb_set(1, 1);
    sccb_set(0, 1);
}

static void sccb_stop(void)
{
    sccb_set(0, 1);
    sccb_set(1, 1);
    sccb_set(1, 0);
}

static int sccb_write_byte(u8 value)
{
    int bit;
    int ack;

    for (bit = 7; bit >= 0; bit--) {
        int drive_low = ((value >> bit) & 0x01U) == 0U;
        sccb_set(0, drive_low);
        sccb_set(1, drive_low);
        sccb_set(0, drive_low);
    }

    sccb_set(0, 0);
    sccb_set(1, 0);
    ack = sccb_read_siod();
    sccb_set(0, 0);

    return ack == 0;
}

static int ov7670_write_reg(u8 reg, u8 value)
{
    int ok;

    sccb_start();
    ok = sccb_write_byte(OV7670_SCCB_ADDR);
    ok = sccb_write_byte(reg) && ok;
    ok = sccb_write_byte(value) && ok;
    sccb_stop();
    usleep(1000);

    return ok;
}

static int ov7670_init_rgb565_qvga(void)
{
    int i;
    int failed = 0;

    for (i = 0; ov7670_rgb565_qvga_init[i].reg != 0xFFU ||
                ov7670_rgb565_qvga_init[i].value != 0x00U; i++) {
        if (ov7670_rgb565_qvga_init[i].reg == 0xFFU &&
            ov7670_rgb565_qvga_init[i].value == 0xFFU) {
            usleep(100000);
            continue;
        }

        if (!ov7670_write_reg(ov7670_rgb565_qvga_init[i].reg,
                              ov7670_rgb565_qvga_init[i].value)) {
            failed++;
        }
    }

    return failed;
}

static void camera_reset_release(void)
{
    camera_write_control(CAM_SIOC);
    usleep(50000);
    camera_write_control(CAM_RST | CAM_SIOC);
    usleep(300000);
}

static void print_input_sample(const char *label)
{
    u32 input = camera_read_input();
    u32 status = camera_read_status();
    u32 color = camera_read_color();

    xil_printf("%s input=0x%08x data=0x%02x vs=%d hs=%d pclk=%d siod=%d\r\n",
               label,
               (unsigned int)input,
               (unsigned int)(input & 0xFFU),
               bit_from(input, CAM_VSYNC_BIT),
               bit_from(input, CAM_HREF_BIT),
               bit_from(input, CAM_PCLK_BIT),
               bit_from(input, CAM_SIOD_BIT));

    xil_printf("    status=0x%08x rgb=(%d,%d,%d) result=%d active=%d byte=%d\r\n",
               (unsigned int)status,
               (int)(status & 0xFFU),
               (int)((status >> 8) & 0xFFU),
               (int)((status >> 16) & 0xFFU),
               (int)((status >> 24) & 0x03U),
               (int)((status >> 26) & 0x01U),
               (int)((status >> 27) & 0x01U));

    xil_printf("    color =0x%08x sky=%d orange=%d\r\n",
               (unsigned int)color,
               (int)(color & 0xFFFFU),
               (int)((color >> 16) & 0xFFFFU));
}

static void poll_camera_signals(void)
{
    u32 i;
    u32 input;
    u32 data;
    u32 previous_data;
    u32 data_change_count = 0U;
    u32 vsync_transition_count = 0U;
    u32 href_transition_count = 0U;
    u32 pclk_transition_count = 0U;
    u32 vsync_high_count = 0U;
    u32 href_high_count = 0U;
    u32 pclk_high_count = 0U;
    u32 min_data = 0xFFU;
    u32 max_data = 0x00U;
    int previous_vsync;
    int previous_href;
    int previous_pclk;

    input = camera_read_input();
    previous_vsync = bit_from(input, CAM_VSYNC_BIT);
    previous_href = bit_from(input, CAM_HREF_BIT);
    previous_pclk = bit_from(input, CAM_PCLK_BIT);
    previous_data = input & 0xFFU;

    for (i = 0U; i < FAST_POLL_COUNT; i++)
    {
        int vsync;
        int href;
        int pclk;

        input = camera_read_input();
        data = input & 0xFFU;
        vsync = bit_from(input, CAM_VSYNC_BIT);
        href = bit_from(input, CAM_HREF_BIT);
        pclk = bit_from(input, CAM_PCLK_BIT);

        if (vsync != 0) {
            vsync_high_count++;
        }
        if (href != 0) {
            href_high_count++;
        }
        if (pclk != 0) {
            pclk_high_count++;
        }
        if (data < min_data) {
            min_data = data;
        }
        if (data > max_data) {
            max_data = data;
        }
        if (data != previous_data) {
            data_change_count++;
            previous_data = data;
        }

        if (vsync != previous_vsync) {
            vsync_transition_count++;
            previous_vsync = vsync;
        }
        if (href != previous_href) {
            href_transition_count++;
            previous_href = href;
        }
        if (pclk != previous_pclk) {
            pclk_transition_count++;
            previous_pclk = pclk;
        }

        if ((vsync_transition_count >= MAX_TRANSITIONS) &&
            (href_transition_count >= MAX_TRANSITIONS) &&
            (pclk_transition_count >= MAX_TRANSITIONS)) {
            break;
        }
    }

    xil_printf("VSYNC transitions = %d, high samples = %d\r\n",
               (int)vsync_transition_count, (int)vsync_high_count);
    xil_printf("HREF  transitions = %d, high samples = %d\r\n",
               (int)href_transition_count, (int)href_high_count);
    xil_printf("PCLK  transitions = %d, high samples = %d\r\n",
               (int)pclk_transition_count, (int)pclk_high_count);
    xil_printf("DATA  changes     = %d, min/max = 0x%02x / 0x%02x\r\n",
               (int)data_change_count, (unsigned int)min_data, (unsigned int)max_data);
}

static void poll_pl_capture_status(void)
{
    int i;

    xil_printf("\r\nPL capture status samples:\r\n");
    for (i = 0; i < 20; i++)
    {
        print_input_sample("sample");
        usleep(100000);
    }
}

static u32 pulse_width_for_result(u32 result)
{
    if (result == 1U) {
        return SERVO_SKY_US;
    }
    if (result == 2U) {
        return SERVO_ORANGE_US;
    }
    return SERVO_BLOCK_US;
}

static u32 detect_camera_result(void)
{
    int i;
    u32 best = 3U;

    for (i = 0; i < DECISION_SAMPLES; i++) {
        u32 color = camera_read_color();
        u32 sky_count = color & 0xFFFFU;
        u32 orange_count = (color >> 16) & 0xFFFFU;

        if ((sky_count > COLOR_MIN_COUNT) &&
            (sky_count >= orange_count)) {
            best = 1U;
        } else if (orange_count > COLOR_MIN_COUNT) {
            best = 2U;
        }
        usleep(100000);
    }

    xil_printf("detect: valid_color=%d -> result=%d\r\n",
               (int)(best != 3U), (int)best);

    return best;
}

static void servo_drive_for_ms(u32 high_us, u32 hold_ms)
{
    u32 cycles = hold_ms / 20U;
    u32 i;

    for (i = 0U; i < cycles; i++) {
        servo_write(1);
        usleep(high_us);
        servo_write(0);
        usleep(SERVO_PERIOD_US - high_us);
    }
}

static void run_sorter_loop(void)
{
    u32 last_result = 0U;

    servo_drive_for_ms(SERVO_BLOCK_US, 1000U);

    while (1) {
        u32 result = detect_camera_result();
        u32 color = camera_read_color();
        u32 high_us = pulse_width_for_result(result);

        if (result != last_result) {
            if (result == 1U) {
                xil_printf("decision: SKY_BLUE -> servo %d us\r\n", (int)high_us);
            } else if (result == 2U) {
                xil_printf("decision: ORANGE -> servo %d us\r\n", (int)high_us);
            } else {
                xil_printf("decision: BLOCK -> servo %d us\r\n", (int)high_us);
            }
            last_result = result;
        }

        xil_printf("counts: sky=%d orange=%d\r\n",
                   (int)(color & 0xFFFFU),
                   (int)((color >> 16) & 0xFFFFU));

        /*
         * Preparation step: move the sub motor to the selected position
         * immediately after the color decision.  The object arrives only
         * after the preparation delay, so the motor is already in position
         * when sorting takes place.
         */
        xil_printf("Preparing servo at %d us...\r\n", (int)high_us);
        servo_drive_for_ms(high_us, 600U);

        xil_printf("Waiting 10 seconds for object arrival...\r\n");
        usleep(10000000U);
    }
}

int main(void)
{
    xil_printf("\r\n\r\n");
    xil_printf("====================================\r\n");
    xil_printf(" AXI SMART SORTER NO-FIFO OV7670 TEST\r\n");
    xil_printf(" Zybo Z7-20\r\n");
    xil_printf("====================================\r\n");
    xil_printf("CAMERA_BASE = 0x%08x\r\n", (unsigned int)CAMERA_BASE);

    print_input_sample("Before reset");
    camera_reset_release();
    print_input_sample("After reset ");

    xil_printf("\r\nOV7670 SCCB init RGB565/QVGA...\r\n");
    {
        int failed = ov7670_init_rgb565_qvga();
        if (failed == 0) {
            xil_printf("OV7670 initialization OK\r\n");
        } else {
            xil_printf("OV7670 initialization completed with %d SCCB NACK(s)\r\n", failed);
        }
    }
    usleep(300000);
    print_input_sample("After init  ");

    xil_printf("\r\nFast raw signal poll:\r\n");
    poll_camera_signals();
    poll_pl_capture_status();

    xil_printf("\r\nStarting sorter loop\r\n");
    run_sorter_loop();

    return 0;
}
