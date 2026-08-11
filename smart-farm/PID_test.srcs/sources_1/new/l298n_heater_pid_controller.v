`timescale 1ns / 1ps

module l298n_heater_controller #(
    parameter integer CLK_HZ = 100_000_000,

    // PI 제어 목표 온도
    parameter [7:0] SET_TEMP_C = 8'd30,

    // 이 온도 이상이면 히터 강제 OFF
    // 목표 온도보다 1°C 낮은 온도에서 OFF하여 잔열로 목표 온도까지 상승
    parameter [7:0] CUTOFF_TEMP_C = 8'd29,

    // 이 온도 이하이면 히터 다시 ON 허용  
    // 목표 온도보다 2°C 낮아지면 다시 ON 허용
    parameter [7:0] RESTART_TEMP_C = 8'd28,

    // ============================================================
    // PI 제어 이득
    // ============================================================

    // 비례 이득
    //
    // 오차 1°C -> 80  -> 8%
    // 오차 2°C -> 160 -> 16%
    // 오차 3°C -> 240 -> 24%
    parameter signed [15:0] KP = 16'sd80,

    // 적분 이득
    //
    // PI 계산 주기마다:
    // I 출력 = 이전 I 출력 + KI × 온도 오차
    //
    // KI = 10일 때:
    // 오차 1°C -> 측정할 때마다 10 증가
    // 오차 2°C -> 측정할 때마다 20 증가
    parameter signed [15:0] KI = 16'sd10,

    // ============================================================
    // 적분 출력 제한
    //
    // 히터는 가열만 가능하므로 기본 최소값은 0으로 설정한다.
    //
    // 100은 PWM Duty 기준 10%에 해당한다.
    // P 출력과 I 출력을 합친 뒤 최종 출력은 25%로 제한된다.
    // ============================================================
    parameter signed [31:0] INTEGRAL_MIN = 32'sd0,
    parameter signed [31:0] INTEGRAL_MAX = 32'sd100,

    // 최대 PWM Duty
    //
    // 250  = 25%
    // 300  = 30%
    // 400  = 40%
    // 1000 = 100%
    parameter integer MAX_DUTY_PERMILLE = 250,

    // PWM 주기
    // 1000ms = 1초
    parameter integer PWM_PERIOD_MS = 1000,

    // PI 계산 주기
    // DHT11 온도 갱신 주기에 맞춰 2000ms마다 한 번 계산
    parameter integer PI_SAMPLE_MS = 2000
)(
    input  wire       clk,
    input  wire       rst,

    // DHT11에서 정상 온도 데이터를 받은 상태
    input  wire       valid,

    // DHT11 센서 오류
    input  wire       error,

    // 현재 온도
    input  wire [7:0] temperature,

    // L298N IN1으로 나가는 PI 제어 PWM
    output reg        l298n_in1
);

    // ============================================================
    // PWM 설정
    //
    // PWM 한 주기를 1000개의 슬롯으로 나눈다.
    //
    // duty_permille:
    //   0   -> 0%
    //   160 -> 16%
    //   250 -> 25%
    //   300 -> 30%
    // ============================================================

    localparam integer PWM_SLOT_COUNT = 1000;

    localparam integer PWM_PERIOD_COUNT =
        (CLK_HZ / 1000) * PWM_PERIOD_MS;

    localparam integer PWM_STEP_COUNT =
        PWM_PERIOD_COUNT / PWM_SLOT_COUNT;

    localparam integer PI_SAMPLE_COUNT =
        (CLK_HZ / 1000) * PI_SAMPLE_MS;

    // PWM 슬롯 위치: 0~999
    reg [9:0] pwm_slot;

    // 한 PWM 슬롯을 유지하는 Clock 카운터
    reg [31:0] pwm_step_counter;

    // 최종 PWM Duty: 0~MAX_DUTY_PERMILLE
    reg [10:0] duty_permille;

    // 히터 동작 허용 상태
    reg heater_allowed;

    // 정상 센서 데이터를 한 번 이상 받았는지 확인
    reg sensor_seen;

    // PI 계산 주기 카운터
    reg [31:0] pi_sample_counter;
    reg        pi_sample_tick;

    // valid가 계속 1로 유지되어도 PI 계산은 주기마다 한 번만 수행
    wire pi_control_update;

    assign pi_control_update =
        valid && (!sensor_seen || pi_sample_tick);

    // PI 계산을 여러 Clock으로 나누기 위한 상태
    reg pi_multiply_pending;
    reg pi_integral_pending;
    reg pi_limit_pending;
    reg pi_duty_pending;
    reg pi_output_pending;

    // PI 계산용 온도 오차 저장값
    reg signed [15:0] sampled_temperature_error;

    // ============================================================
    // I 제어 적분 누적값
    //
    // integral_acc는 이미 PWM Duty 단위로 계산된 I 출력이다.
    //
    // integral_acc[k]
    // = integral_acc[k-1] + KI × error[k]
    // ============================================================

    reg signed [31:0] integral_acc;

    // ============================================================
    // 온도 오차 계산
    //
    // temperature_error = 목표 온도 - 현재 온도
    // ============================================================

    wire signed [15:0] set_temperature_signed;
    wire signed [15:0] current_temperature_signed;
    wire signed [15:0] temperature_error;

    assign set_temperature_signed =
        $signed({8'd0, SET_TEMP_C});

    assign current_temperature_signed =
        $signed({8'd0, temperature});

    assign temperature_error =
        set_temperature_signed - current_temperature_signed;

    // ============================================================
    // P 출력 계산
    //
    // P = KP × error
    // ============================================================

    reg signed [31:0] p_output;

    // ============================================================
    // I 증가량 계산
    //
    // I 증가량 = KI × error
    //
    // pi_control_update가 들어올 때만 실제 integral_acc에 반영한다.
    // ============================================================

    reg signed [31:0] i_increment;

    // 적분 누적 후보값
    reg signed [31:0] integral_candidate;

    // ============================================================
    // 적분값 제한 함수
    //
    // 적분값이 계속 증가하는 Integral Windup을 막는다.
    // ============================================================

    function signed [31:0] limit_integral;

        input signed [31:0] value;

        begin
            if (value <= INTEGRAL_MIN) begin
                limit_integral = INTEGRAL_MIN;
            end
            else if (value >= INTEGRAL_MAX) begin
                limit_integral = INTEGRAL_MAX;
            end
            else begin
                limit_integral = value;
            end
        end

    endfunction

    reg signed [31:0] limited_integral;

    // ============================================================
    // PI 출력 계산
    //
    // PI 출력 = P 출력 + 제한된 I 출력
    // ============================================================

    reg signed [31:0] pi_output;

    // ============================================================
    // 최종 PWM Duty 제한 함수
    //
    // PI 계산 결과를 0~MAX_DUTY_PERMILLE로 제한한다.
    // 기본 최대값은 250이므로 최대 PWM은 25%이다.
    // ============================================================

    function [10:0] limit_duty;

        input signed [31:0] value;

        begin
            if (value <= 32'sd0) begin
                limit_duty = 11'd0;
            end
            else if (value >= MAX_DUTY_PERMILLE) begin
                limit_duty = MAX_DUTY_PERMILLE;
            end
            else begin
                limit_duty = value[10:0];
            end
        end

    endfunction

    // ============================================================
    // PI 계산 주기 생성
    //
    // data_valid가 한 번 정상 수신 뒤 계속 1로 유지되더라도
    // 적분값은 PI_SAMPLE_MS마다 한 번만 갱신한다.
    // ============================================================

    always @(posedge clk) begin
        if (rst) begin
            pi_sample_counter <= 32'd0;
            pi_sample_tick    <= 1'b0;
        end
        else begin
            pi_sample_tick <= 1'b0;

            if (error || !valid) begin
                pi_sample_counter <= 32'd0;
            end
            else begin
                if (pi_sample_counter >= PI_SAMPLE_COUNT - 1) begin
                    pi_sample_counter <= 32'd0;
                    pi_sample_tick    <= 1'b1;
                end
                else begin
                    pi_sample_counter <= pi_sample_counter + 1'b1;
                end
            end
        end
    end

    // ============================================================
    // 온도 히스테리시스 및 PI 제어
    //
    // temperature >= CUTOFF_TEMP_C
    //     -> 히터 강제 OFF
    //     -> 적분값 초기화
    //
    // temperature <= RESTART_TEMP_C
    //     -> 히터 동작 허용
    //     -> PI 출력 계산
    //
    // 두 온도 사이
    //     -> 이전 heater_allowed 상태 유지
    // ============================================================

    always @(posedge clk) begin
        if (rst) begin
            heater_allowed <= 1'b0;
            sensor_seen    <= 1'b0;
            duty_permille  <= 11'd0;
            integral_acc   <= 32'sd0;
            pi_multiply_pending      <= 1'b0;
            pi_integral_pending      <= 1'b0;
            pi_limit_pending         <= 1'b0;
            pi_duty_pending          <= 1'b0;
            pi_output_pending        <= 1'b0;
            sampled_temperature_error <= 16'sd0;
            p_output                 <= 32'sd0;
            i_increment              <= 32'sd0;
            integral_candidate       <= 32'sd0;
            limited_integral         <= 32'sd0;
            pi_output                <= 32'sd0;
        end
        else begin

            // 센서 오류 발생 시 안전을 위해 즉시 OFF
            if (error) begin
                heater_allowed <= 1'b0;
                sensor_seen    <= 1'b0;
                duty_permille  <= 11'd0;
                integral_acc   <= 32'sd0;
                pi_multiply_pending <= 1'b0;
                pi_integral_pending <= 1'b0;
                pi_limit_pending    <= 1'b0;
                pi_duty_pending     <= 1'b0;
                pi_output_pending   <= 1'b0;
            end

            else begin

                // P 출력과 I 증가량 계산
                if (pi_multiply_pending) begin
                    p_output <= sampled_temperature_error * KP;
                    i_increment <= sampled_temperature_error * KI;

                    pi_multiply_pending <= 1'b0;
                    pi_integral_pending <= 1'b1;
                end

                // 적분 누적 후보값 계산
                if (pi_integral_pending) begin
                    integral_candidate <= integral_acc + i_increment;

                    pi_integral_pending <= 1'b0;
                    pi_limit_pending    <= 1'b1;
                end

                // 적분값 제한
                if (pi_limit_pending) begin
                    limited_integral <= limit_integral(integral_candidate);

                    pi_limit_pending <= 1'b0;
                    pi_duty_pending  <= 1'b1;
                end

                // P + I 결과를 PWM Duty로 변환
                if (pi_duty_pending) begin
                    pi_output <= p_output + limited_integral;

                    pi_duty_pending <= 1'b0;
                    pi_output_pending <= 1'b1;
                end

                // 최종 PWM Duty 제한
                if (pi_output_pending) begin
                    integral_acc <= limited_integral;
                    duty_permille <= limit_duty(pi_output);

                    pi_output_pending <= 1'b0;
                end

                // 새로운 정상 온도 데이터가 들어온 경우 또는
                // PI 계산 주기가 된 경우에만
                // PI 제어값을 갱신한다.
                if (pi_control_update) begin
                    sensor_seen <= 1'b1;

                    // 차단 온도 이상이면 히터 강제 OFF
                    if (temperature >= CUTOFF_TEMP_C) begin
                        heater_allowed <= 1'b0;
                        duty_permille  <= 11'd0;

                        // 히터가 꺼진 상태에서 적분값이 남아 있으면
                        // 다시 켜질 때 출력이 갑자기 커질 수 있으므로 초기화
                        integral_acc <= 32'sd0;

                        pi_multiply_pending <= 1'b0;
                        pi_integral_pending <= 1'b0;
                        pi_limit_pending    <= 1'b0;
                        pi_duty_pending     <= 1'b0;
                        pi_output_pending   <= 1'b0;
                    end

                    // 재시작 온도 이하이면 PI 제어 시작
                    else if (temperature <= RESTART_TEMP_C) begin
                        heater_allowed <= 1'b1;
                        sampled_temperature_error <= temperature_error;

                        pi_multiply_pending <= 1'b1;
                        pi_integral_pending <= 1'b0;
                        pi_limit_pending    <= 1'b0;
                        pi_duty_pending     <= 1'b0;
                        pi_output_pending   <= 1'b0;
                    end

                    // 히스테리시스 구간
                    else begin
                        if (heater_allowed) begin
                            sampled_temperature_error <= temperature_error;

                            pi_multiply_pending <= 1'b1;
                            pi_integral_pending <= 1'b0;
                            pi_limit_pending    <= 1'b0;
                            pi_duty_pending     <= 1'b0;
                            pi_output_pending   <= 1'b0;
                        end
                        else begin
                            duty_permille <= 11'd0;
                            integral_acc  <= 32'sd0;

                            pi_multiply_pending <= 1'b0;
                            pi_integral_pending <= 1'b0;
                            pi_limit_pending    <= 1'b0;
                            pi_duty_pending     <= 1'b0;
                            pi_output_pending   <= 1'b0;
                        end
                    end
                end
            end
        end
    end

    // ============================================================
    // 0~999 PWM 슬롯 카운터
    // ============================================================

    always @(posedge clk) begin
        if (rst) begin
            pwm_step_counter <= 32'd0;
            pwm_slot         <= 10'd0;
        end
        else begin

            // 히터가 꺼져 있으면 PWM 카운터 초기화
            if (!heater_allowed ||
                !sensor_seen     ||
                error            ||
                (duty_permille == 11'd0)) begin

                pwm_step_counter <= 32'd0;
                pwm_slot         <= 10'd0;
            end

            else begin
                if (pwm_step_counter >= PWM_STEP_COUNT - 1) begin
                    pwm_step_counter <= 32'd0;

                    if (pwm_slot >= 10'd999) begin
                        pwm_slot <= 10'd0;
                    end
                    else begin
                        pwm_slot <= pwm_slot + 1'b1;
                    end
                end
                else begin
                    pwm_step_counter <= pwm_step_counter + 1'b1;
                end
            end
        end
    end

    // ============================================================
    // PI 제어 PWM 출력
    //
    // duty_permille = 250일 때:
    //
    // pwm_slot 0~249   -> HIGH
    // pwm_slot 250~999 -> LOW
    //
    // 따라서 최대 Duty는 25%
    // ============================================================

    always @(*) begin
        if (heater_allowed &&
            sensor_seen     &&
            !error          &&
            (duty_permille != 11'd0) &&
            ({1'b0, pwm_slot} < duty_permille)) begin

            l298n_in1 = 1'b1;
        end
        else begin
            l298n_in1 = 1'b0;
        end
    end

endmodule
