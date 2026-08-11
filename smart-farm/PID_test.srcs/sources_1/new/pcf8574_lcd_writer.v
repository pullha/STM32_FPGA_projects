`timescale 1ns / 1ps

module pcf8574_lcd_writer #(
    parameter integer CLK_HZ   = 100_000_000,
    parameter integer I2C_HZ   = 100_000,
    parameter [6:0]   I2C_ADDR = 7'h27
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    input  wire       rs,
    input  wire       nibble_only,
    input  wire [7:0] lcd_data,
    output reg        busy,
    output reg        done,
    output wire       ack_error,
    inout  wire       i2c_scl,
    inout  wire       i2c_sda
);

    reg        i2c_start;
    reg [7:0]  i2c_data;
    wire       i2c_busy;
    wire       i2c_done;
    wire       i2c_ack_error;

    i2c_master_write #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ),
        .I2C_ADDR(I2C_ADDR)
    ) u_i2c_master_write (
        .clk(clk),
        .rst(rst),
        .start(i2c_start),
        .data_in(i2c_data),
        .busy(i2c_busy),
        .done(i2c_done),
        .ack_error(i2c_ack_error),
        .i2c_scl(i2c_scl),
        .i2c_sda(i2c_sda)
    );

    assign ack_error = i2c_ack_error;

    reg       rs_latched;
    reg       nibble_only_latched;
    reg [7:0] data_latched;
    reg [2:0] step;
    reg       waiting_i2c;

    function [7:0] make_pcf_byte;
        input [3:0] nibble;
        input       enable_bit;
        input       rs_bit;
        begin
            // PCF8574 mapping used by most 1602 I2C backpacks:
            // P7..P4 = LCD D7..D4, P3 = Backlight,
            // P2 = E, P1 = RW, P0 = RS
            make_pcf_byte = {nibble, 1'b1, enable_bit, 1'b0, rs_bit};
        end
    endfunction

    function [7:0] step_byte;
        input [2:0] step_in;
        input [7:0] data_in_f;
        input       rs_in_f;
        begin
            case (step_in)
                3'd0: step_byte = make_pcf_byte(data_in_f[7:4], 1'b0, rs_in_f);
                3'd1: step_byte = make_pcf_byte(data_in_f[7:4], 1'b1, rs_in_f);
                3'd2: step_byte = make_pcf_byte(data_in_f[7:4], 1'b0, rs_in_f);
                3'd3: step_byte = make_pcf_byte(data_in_f[3:0], 1'b0, rs_in_f);
                3'd4: step_byte = make_pcf_byte(data_in_f[3:0], 1'b1, rs_in_f);
                3'd5: step_byte = make_pcf_byte(data_in_f[3:0], 1'b0, rs_in_f);
                default: step_byte = 8'h08;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            i2c_start            <= 1'b0;
            i2c_data             <= 8'd0;
            busy                 <= 1'b0;
            done                 <= 1'b0;
            rs_latched           <= 1'b0;
            nibble_only_latched  <= 1'b0;
            data_latched         <= 8'd0;
            step                 <= 3'd0;
            waiting_i2c          <= 1'b0;
        end else begin
            i2c_start <= 1'b0;
            done      <= 1'b0;

            if (!busy) begin
                if (start) begin
                    busy                <= 1'b1;
                    rs_latched          <= rs;
                    nibble_only_latched <= nibble_only;
                    data_latched        <= lcd_data;
                    step                <= 3'd0;
                    waiting_i2c         <= 1'b0;
                end
            end else begin
                if (!waiting_i2c && !i2c_busy) begin
                    i2c_data    <= step_byte(step, data_latched, rs_latched);
                    i2c_start   <= 1'b1;
                    waiting_i2c <= 1'b1;
                end

                if (waiting_i2c && i2c_done) begin
                    waiting_i2c <= 1'b0;

                    if ((nibble_only_latched && step == 3'd2) ||
                        (!nibble_only_latched && step == 3'd5)) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                    end else begin
                        step <= step + 1'b1;
                    end
                end
            end
        end
    end

endmodule