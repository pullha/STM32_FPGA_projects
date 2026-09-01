`timescale 1ns / 1ps

// ============================================================
// 1us tick generator
// 100MHz clock 기준으로 100 clock마다 tick을 1 clock 동안 1로 만든다.
// ============================================================
module tick_1us_gen #(
    parameter CLK_HZ = 100000000
)(
    input  wire clk,
    input  wire rst,
    output reg  tick
);

    localparam DIV_COUNT = CLK_HZ / 1000000;

    reg [31:0] div_counter;

    always @(posedge clk) begin
        if (rst) begin
            div_counter <= 32'd0;
            tick <= 1'b0;
        end else begin
            if (div_counter == DIV_COUNT - 1) begin
                div_counter <= 32'd0;
                tick <= 1'b1;
            end else begin
                div_counter <= div_counter + 1'b1;
                tick <= 1'b0;
            end
        end
    end

endmodule