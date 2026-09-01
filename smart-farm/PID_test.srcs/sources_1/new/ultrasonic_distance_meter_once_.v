`timescale 1ns/1ps

module ultrasonic_distance_meter_once #(
    parameter CLK_FREQ_HZ     = 100_000_000,
    parameter TRIG_PULSE_US   = 10,
    parameter ECHO_TIMEOUT_US = 30_000
)(
    input  wire       clk,
    input  wire       reset,
    input  wire       start,

    input  wire       echo,
    output reg        trig,

    output reg [9:0]  distance_cm,
    output reg        distance_valid,
    output reg        busy,
    output reg        timeout_error
);

localparam CYCLES_PER_US = CLK_FREQ_HZ / 1_000_000;

localparam S_IDLE      = 3'd0;
localparam S_TRIG      = 3'd1;
localparam S_WAIT_ECHO = 3'd2;
localparam S_MEASURE   = 3'd3;
localparam S_DONE      = 3'd4;
localparam S_CALC      = 3'd5;

reg [2:0] state = S_IDLE;

reg [31:0] us_div_cnt = 0;
reg tick_1us = 0;

reg [31:0] trig_us_cnt = 0;
reg [31:0] wait_us_cnt = 0;
reg [31:0] echo_us_cnt = 0;
reg [31:0] calc_us_left = 0;
reg [9:0]  calc_cm = 0;

reg echo_meta = 0;
reg echo_sync = 0;
reg echo_old  = 0;

wire echo_rise = echo_sync & ~echo_old;
wire echo_fall = ~echo_sync & echo_old;

always @(posedge clk) begin
    if(reset) begin
        echo_meta <= 0;
        echo_sync <= 0;
        echo_old  <= 0;
    end else begin
        echo_meta <= echo;
        echo_sync <= echo_meta;
        echo_old  <= echo_sync;
    end
end

always @(posedge clk) begin
    tick_1us <= 1'b0;

    if(reset || state == S_IDLE) begin
        us_div_cnt <= 0;
    end else begin
        if(us_div_cnt >= CYCLES_PER_US - 1) begin
            us_div_cnt <= 0;
            tick_1us <= 1'b1;
        end else begin
            us_div_cnt <= us_div_cnt + 1;
        end
    end
end

always @(posedge clk) begin
    if(reset) begin
        state <= S_IDLE;
        trig <= 0;
        busy <= 0;
        distance_cm <= 0;
        distance_valid <= 0;
        timeout_error <= 0;
        trig_us_cnt <= 0;
        wait_us_cnt <= 0;
        echo_us_cnt <= 0;
        calc_us_left <= 0;
        calc_cm <= 0;
    end else begin
        distance_valid <= 0;
        timeout_error <= 0;

        case(state)
            S_IDLE: begin
                trig <= 0;
                busy <= 0;

                if(start) begin
                    busy <= 1;
                    trig_us_cnt <= 0;
                    wait_us_cnt <= 0;
                    echo_us_cnt <= 0;
                    calc_us_left <= 0;
                    calc_cm <= 0;
                    state <= S_TRIG;
                end
            end

            S_TRIG: begin
                busy <= 1;
                trig <= 1;

                if(tick_1us) begin
                    if(trig_us_cnt >= TRIG_PULSE_US - 1) begin
                        trig <= 0;
                        wait_us_cnt <= 0;
                        state <= S_WAIT_ECHO;
                    end else begin
                        trig_us_cnt <= trig_us_cnt + 1;
                    end
                end
            end

            S_WAIT_ECHO: begin
                busy <= 1;
                trig <= 0;

                if(echo_rise) begin
                    echo_us_cnt <= 1;
                    state <= S_MEASURE;
                end else if(tick_1us) begin
                    if(wait_us_cnt >= ECHO_TIMEOUT_US) begin
                        timeout_error <= 1;
                        busy <= 0;
                        state <= S_IDLE;
                    end else begin
                        wait_us_cnt <= wait_us_cnt + 1;
                    end
                end
            end

            S_MEASURE: begin
                busy <= 1;

                if(echo_fall) begin
                    calc_us_left <= echo_us_cnt;
                    calc_cm <= 0;
                    state <= S_CALC;
                end else if(tick_1us) begin
                    if(echo_us_cnt >= ECHO_TIMEOUT_US) begin
                        timeout_error <= 1;
                        busy <= 0;
                        state <= S_IDLE;
                    end else begin
                        echo_us_cnt <= echo_us_cnt + 1;
                    end
                end
            end

            S_CALC: begin
                busy <= 1;

                if(calc_us_left >= 32'd58 && calc_cm < 10'd999) begin
                    calc_us_left <= calc_us_left - 32'd58;
                    calc_cm <= calc_cm + 1'b1;
                end else begin
                    distance_cm <= calc_cm;
                    distance_valid <= 1;
                    busy <= 0;
                    state <= S_IDLE;
                end
            end

            default: begin
                state <= S_IDLE;
            end
        endcase
    end
end

endmodule
