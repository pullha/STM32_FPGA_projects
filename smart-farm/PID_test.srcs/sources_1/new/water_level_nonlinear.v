`timescale 1ns / 1ps

module water_level_nonlinear(
    input  wire        clk,
    input  wire [11:0] water_adc,

    output reg  [3:0]  an,
    output reg  [6:0]  seg
);

    reg [23:0] filter_reg = 24'd0;

    always @(posedge clk) begin
        filter_reg <= filter_reg - (filter_reg >> 10) + water_adc;
    end

    wire [11:0] stable_adc = filter_reg[21:10];

    localparam [11:0] THRES_0 = 12'd100;
    localparam [11:0] THRES_1 = 12'd1700;
    localparam [11:0] THRES_2 = 12'd1775;
    localparam [11:0] THRES_3 = 12'd1800;

    reg [3:0] water_level = 4'd0;

    always @(*) begin
        if      (stable_adc < THRES_0) water_level = 4'd0;
        else if (stable_adc < THRES_1) water_level = 4'd1;
        else if (stable_adc < THRES_2) water_level = 4'd2;
        else if (stable_adc < THRES_3) water_level = 4'd3;
        else                           water_level = 4'd4;
    end

    reg [19:0] refresh_cnt = 20'd0;
    wire [1:0] digit_sel;

    always @(posedge clk) begin
        refresh_cnt <= refresh_cnt + 1'b1;
    end

    assign digit_sel = refresh_cnt[19:18];

    // Basys3 7-segment active-low, seg[6:0] = {g, f, e, d, c, b, a}
    function [6:0] seg_digit;
        input [3:0] num;
        begin
            case (num)
                4'd0: seg_digit = 7'b1000000;
                4'd1: seg_digit = 7'b1111001;
                4'd2: seg_digit = 7'b0100100;
                4'd3: seg_digit = 7'b0110000;
                4'd4: seg_digit = 7'b0011001;
                4'd5: seg_digit = 7'b0010010;
                4'd6: seg_digit = 7'b0000010;
                4'd7: seg_digit = 7'b1111000;
                4'd8: seg_digit = 7'b0000000;
                4'd9: seg_digit = 7'b0011000;
                default: seg_digit = 7'b1111111;
            endcase
        end
    endfunction

    localparam [6:0] SEG_BLANK = 7'b1111111;

    always @(*) begin
        an  = 4'b1111;
        seg = SEG_BLANK;

        case (digit_sel)
            2'd0: begin
                an  = 4'b1110;
                seg = seg_digit(water_level);
            end

            2'd1: begin
                an  = 4'b1101;
                seg = SEG_BLANK;
            end

            2'd2: begin
                an  = 4'b1111;
                seg = SEG_BLANK;
            end

            2'd3: begin
                an  = 4'b1111;
                seg = SEG_BLANK;
            end

            default: begin
                an  = 4'b1111;
                seg = SEG_BLANK;
            end
        endcase
    end

endmodule
