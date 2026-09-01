`timescale 1ns / 1ps

// ============================================================
// L298 Humidity Fan Controller
//
// humidity < 80%   -> 모터 정지
// humidity = 80%   -> 약 50% duty로 회전 시작
// humidity = 100%  -> 100% duty 최고 속도
//
// L298 연결:
// fan_ena_pwm -> L298 ENA
// fan_in1     -> L298 IN1
// fan_in2     -> L298 IN2
// ============================================================
module l298_humidity_fan_controller #(
    parameter integer CLK_FREQ_HZ       = 100000000,
    parameter integer PWM_FREQ_HZ       = 1000,

    // 모터가 돌기 시작하는 습도
    parameter [7:0] HUM_START_PERCENT   = 8'd80,

    // 최고 속도에 도달하는 습도
    parameter [7:0] HUM_FULL_PERCENT    = 8'd100,

    // 80%에서 시작할 최소 duty
    // DC 모터는 duty가 너무 낮으면 안 돌 수 있어서 50% 정도로 시작한다.
    parameter [7:0] MIN_DUTY            = 8'd128
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       valid,
    input  wire [7:0] humidity,

    output wire       fan_ena_pwm,
    output reg        fan_in1,
    output reg        fan_in2,

    // 필요 없으면 top에서 연결하지 않아도 된다.
    output reg [7:0]  fan_duty_debug
);

    localparam integer PWM_PERIOD_COUNT = CLK_FREQ_HZ / PWM_FREQ_HZ;

    localparam integer HUM_RANGE_INT =
        (HUM_FULL_PERCENT > HUM_START_PERCENT) ?
        (HUM_FULL_PERCENT - HUM_START_PERCENT) : 1;

    localparam integer DUTY_RANGE_INT = 255 - MIN_DUTY;

    reg [31:0] pwm_count;
    reg [7:0]  fan_duty;
    reg [15:0] duty_calc;

    wire [31:0] pwm_compare;

    // 1kHz PWM counter
    always @(posedge clk) begin
        if (rst) begin
            pwm_count <= 32'd0;
        end else begin
            if (pwm_count >= PWM_PERIOD_COUNT - 1)
                pwm_count <= 32'd0;
            else
                pwm_count <= pwm_count + 1'b1;
        end
    end

    // 습도 기반 duty 계산
    always @(*) begin
        fan_duty = 8'd0;
        duty_calc = 16'd0;

        if (!valid) begin
            fan_duty = 8'd0;
        end else if (humidity < HUM_START_PERCENT) begin
            fan_duty = 8'd0;
        end else if (humidity >= HUM_FULL_PERCENT) begin
            fan_duty = 8'hFF;
        end else begin
            duty_calc =
                MIN_DUTY +
                (((humidity - HUM_START_PERCENT) * DUTY_RANGE_INT) / HUM_RANGE_INT);

            fan_duty = duty_calc[7:0];
        end
    end

    // 환풍기 방향은 한쪽 방향으로 고정한다.
    // 방향이 반대면 fan_in1/fan_in2 값을 서로 바꾸거나 모터 선을 반대로 연결하면 된다.
    always @(*) begin
        if (fan_duty == 8'd0) begin
            fan_in1 = 1'b0;
            fan_in2 = 1'b0;
        end else begin
            fan_in1 = 1'b1;
            fan_in2 = 1'b0;
        end
    end

    always @(*) begin
        fan_duty_debug = fan_duty;
    end

    assign pwm_compare = (PWM_PERIOD_COUNT * fan_duty) >> 8;

    assign fan_ena_pwm =
        (fan_duty == 8'd0)  ? 1'b0 :
        (fan_duty == 8'hFF) ? 1'b1 :
        (pwm_count < pwm_compare);

endmodule