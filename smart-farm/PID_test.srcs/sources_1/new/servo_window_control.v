`timescale 1ns / 1ps

module servo_window_control #(
    parameter CLK_HZ          = 100000000,
    parameter OPEN_TEMP_C     = 25,
    parameter CLOSE_TEMP_C    = 24,
    parameter CLOSED_PULSE_US = 1000,
    parameter OPEN_PULSE_US   = 2000
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       valid,
    input  wire [7:0] temperature,
    input  wire       force_closed,
    output reg        servo_pwm
);

    localparam integer CYCLES_PER_US      = CLK_HZ / 1000000;
    localparam integer SERVO_PERIOD_US    = 20000;
    localparam integer SERVO_PERIOD_COUNT = SERVO_PERIOD_US * CYCLES_PER_US;
    localparam integer CLOSED_PULSE_COUNT = CLOSED_PULSE_US * CYCLES_PER_US;
    localparam integer OPEN_PULSE_COUNT   = OPEN_PULSE_US * CYCLES_PER_US;

    reg [31:0] pwm_count;
    reg        window_open;

    wire [31:0] pulse_count = window_open ? OPEN_PULSE_COUNT : CLOSED_PULSE_COUNT;

    always @(posedge clk) begin
        if (rst) begin
            window_open <= 1'b0;
        end else if (force_closed) begin
            window_open <= 1'b0;
        end else if (valid) begin
            if (temperature >= OPEN_TEMP_C)
                window_open <= 1'b1;
            else if (temperature <= CLOSE_TEMP_C)
                window_open <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            pwm_count <= 32'd0;
            servo_pwm <= 1'b0;
        end else begin
            if (pwm_count >= SERVO_PERIOD_COUNT - 1)
                pwm_count <= 32'd0;
            else
                pwm_count <= pwm_count + 1;
            servo_pwm <= (pwm_count < pulse_count);
        end
    end

endmodule
