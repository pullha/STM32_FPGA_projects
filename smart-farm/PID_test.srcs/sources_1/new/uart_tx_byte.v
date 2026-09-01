`timescale 1ns / 1ps

// ============================================================
// UART TX Byte
// 9600 baud, 8 data bits, no parity, 1 stop bit
// ============================================================
module uart_tx_byte #(
    parameter integer CLK_FREQ = 100000000,
    parameter integer BAUD_RATE = 9600,
    parameter integer CLKS_PER_BIT = (CLK_FREQ + BAUD_RATE / 2) / BAUD_RATE
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data,
    input  wire       start,

    output reg        tx,
    output reg        busy,
    output reg        done
);

    localparam U_IDLE  = 2'd0;
    localparam U_START = 2'd1;
    localparam U_DATA  = 2'd2;
    localparam U_STOP  = 2'd3;

    reg [1:0]  state;
    reg [31:0] bit_count;
    reg [2:0]  bit_index;
    reg [7:0]  data_reg;

    always @(posedge clk) begin
        if (rst) begin
            state     <= U_IDLE;
            bit_count <= 32'd0;
            bit_index <= 3'd0;
            data_reg  <= 8'd0;
            tx        <= 1'b1;
            busy      <= 1'b0;
            done      <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                U_IDLE: begin
                    tx        <= 1'b1;
                    busy      <= 1'b0;
                    bit_count <= 32'd0;
                    bit_index <= 3'd0;

                    if (start) begin
                        data_reg <= data;
                        tx       <= 1'b0;   // start bit
                        busy     <= 1'b1;
                        state    <= U_START;
                    end
                end

                U_START: begin
                    tx   <= 1'b0;
                    busy <= 1'b1;

                    if (bit_count >= CLKS_PER_BIT - 1) begin
                        bit_count <= 32'd0;
                        tx        <= data_reg[0];
                        bit_index <= 3'd0;
                        state     <= U_DATA;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                    end
                end

                U_DATA: begin
                    busy <= 1'b1;

                    if (bit_count >= CLKS_PER_BIT - 1) begin
                        bit_count <= 32'd0;

                        if (bit_index == 3'd7) begin
                            bit_index <= 3'd0;
                            tx        <= 1'b1;   // stop bit
                            state     <= U_STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                            tx        <= data_reg[bit_index + 1'b1];
                        end
                    end else begin
                        bit_count <= bit_count + 1'b1;
                    end
                end

                U_STOP: begin
                    tx   <= 1'b1;
                    busy <= 1'b1;

                    if (bit_count >= CLKS_PER_BIT - 1) begin
                        bit_count <= 32'd0;
                        busy      <= 1'b0;
                        done      <= 1'b1;
                        state     <= U_IDLE;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                    end
                end

                default: begin
                    state     <= U_IDLE;
                    bit_count <= 32'd0;
                    bit_index <= 3'd0;
                    tx        <= 1'b1;
                    busy      <= 1'b0;
                    done      <= 1'b0;
                end
            endcase
        end
    end

endmodule