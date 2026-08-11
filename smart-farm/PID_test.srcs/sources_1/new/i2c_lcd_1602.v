`timescale 1ns / 1ps

module i2c_lcd_1602 #(
    parameter integer CLK_HZ   = 100_000_000,
    parameter integer I2C_HZ   = 100_000,
    parameter [6:0]   I2C_ADDR = 7'h27
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] temperature,
    input  wire [7:0] humidity,
    input  wire       data_valid,
    output wire       ack_error,
    inout  wire       i2c_scl,
    inout  wire       i2c_sda
);

    localparam integer WAIT_50MS  = CLK_HZ / 20;
    localparam integer WAIT_5MS   = CLK_HZ / 200;
    localparam integer WAIT_1MS   = CLK_HZ / 1000;
    localparam integer WAIT_2MS   = CLK_HZ / 500;

    reg        lcd_start;
    reg        lcd_rs;
    reg        lcd_nibble_only;
    reg [7:0]  lcd_data;
    wire       lcd_busy;
    wire       lcd_done;

    pcf8574_lcd_writer #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ),
        .I2C_ADDR(I2C_ADDR)
    ) u_pcf8574_lcd_writer (
        .clk(clk),
        .rst(rst),
        .start(lcd_start),
        .rs(lcd_rs),
        .nibble_only(lcd_nibble_only),
        .lcd_data(lcd_data),
        .busy(lcd_busy),
        .done(lcd_done),
        .ack_error(ack_error),
        .i2c_scl(i2c_scl),
        .i2c_sda(i2c_sda)
    );

    reg [7:0] temp_latched;
    reg [7:0] hum_latched;
    reg       data_toggle;
    reg       handled_toggle;

    always @(posedge clk) begin
        if (rst) begin
            temp_latched <= 8'd0;
            hum_latched  <= 8'd0;
            data_toggle  <= 1'b0;
        end else if (data_valid) begin
            temp_latched <= temperature;
            hum_latched  <= humidity;
            data_toggle  <= ~data_toggle;
        end
    end

    function [7:0] line1_char;
        input [4:0] index;
        begin
            case (index)
                5'd0:  line1_char = "T";
                5'd1:  line1_char = "E";
                5'd2:  line1_char = "M";
                5'd3:  line1_char = "P";
                5'd4:  line1_char = ":";
                5'd5:  line1_char = " ";
                5'd6:  line1_char = (temp_latched < 10) ? " " : (8'h30 + (temp_latched / 10));
                5'd7:  line1_char = 8'h30 + (temp_latched % 10);
                5'd8:  line1_char = " ";
                5'd9:  line1_char = 8'hDF;
                5'd10: line1_char = "C";
                default: line1_char = " ";
            endcase
        end
    endfunction

    function [7:0] line2_char;
        input [4:0] index;
        begin
            case (index)
                5'd0:  line2_char = "H";
                5'd1:  line2_char = "U";
                5'd2:  line2_char = "M";
                5'd3:  line2_char = "I";
                5'd4:  line2_char = ":";
                5'd5:  line2_char = " ";
                5'd6:  line2_char = (hum_latched < 10) ? " " : (8'h30 + (hum_latched / 10));
                5'd7:  line2_char = 8'h30 + (hum_latched % 10);
                5'd8:  line2_char = " ";
                5'd9:  line2_char = "%";
                default: line2_char = " ";
            endcase
        end
    endfunction

    localparam [5:0]
        ST_PWR_DELAY          = 6'd0,
        ST_INIT_3A_SEND       = 6'd1,
        ST_INIT_3A_WAIT       = 6'd2,
        ST_INIT_3A_DELAY      = 6'd3,
        ST_INIT_3B_SEND       = 6'd4,
        ST_INIT_3B_WAIT       = 6'd5,
        ST_INIT_3B_DELAY      = 6'd6,
        ST_INIT_3C_SEND       = 6'd7,
        ST_INIT_3C_WAIT       = 6'd8,
        ST_INIT_3C_DELAY      = 6'd9,
        ST_INIT_2_SEND        = 6'd10,
        ST_INIT_2_WAIT        = 6'd11,
        ST_CMD_28_SEND        = 6'd12,
        ST_CMD_28_WAIT        = 6'd13,
        ST_CMD_08_SEND        = 6'd14,
        ST_CMD_08_WAIT        = 6'd15,
        ST_CMD_01_SEND        = 6'd16,
        ST_CMD_01_WAIT        = 6'd17,
        ST_CMD_01_DELAY       = 6'd18,
        ST_CMD_06_SEND        = 6'd19,
        ST_CMD_06_WAIT        = 6'd20,
        ST_CMD_0C_SEND        = 6'd21,
        ST_CMD_0C_WAIT        = 6'd22,
        ST_READY              = 6'd23,
        ST_LINE1_ADDR_SEND    = 6'd24,
        ST_LINE1_ADDR_WAIT    = 6'd25,
        ST_LINE1_CHAR_SEND    = 6'd26,
        ST_LINE1_CHAR_WAIT    = 6'd27,
        ST_LINE2_ADDR_SEND    = 6'd28,
        ST_LINE2_ADDR_WAIT    = 6'd29,
        ST_LINE2_CHAR_SEND    = 6'd30,
        ST_LINE2_CHAR_WAIT    = 6'd31;

    reg [5:0]  state;
    reg [31:0] delay_count;
    reg [4:0]  char_index;

    always @(posedge clk) begin
        if (rst) begin
            lcd_start       <= 1'b0;
            lcd_rs          <= 1'b0;
            lcd_nibble_only <= 1'b0;
            lcd_data        <= 8'd0;
            state           <= ST_PWR_DELAY;
            delay_count     <= WAIT_50MS - 1;
            char_index      <= 5'd0;
            handled_toggle  <= 1'b0;
        end else begin
            lcd_start <= 1'b0;

            case (state)
                ST_PWR_DELAY: begin
                    if (delay_count == 0)
                        state <= ST_INIT_3A_SEND;
                    else
                        delay_count <= delay_count - 1'b1;
                end

                ST_INIT_3A_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b0;
                        lcd_nibble_only <= 1'b1;
                        lcd_data        <= 8'h30;
                        lcd_start       <= 1'b1;
                        state           <= ST_INIT_3A_WAIT;
                    end
                end

                ST_INIT_3A_WAIT: begin
                    if (lcd_done) begin
                        delay_count <= WAIT_5MS - 1;
                        state       <= ST_INIT_3A_DELAY;
                    end
                end

                ST_INIT_3A_DELAY: begin
                    if (delay_count == 0)
                        state <= ST_INIT_3B_SEND;
                    else
                        delay_count <= delay_count - 1'b1;
                end

                ST_INIT_3B_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b0;
                        lcd_nibble_only <= 1'b1;
                        lcd_data        <= 8'h30;
                        lcd_start       <= 1'b1;
                        state           <= ST_INIT_3B_WAIT;
                    end
                end

                ST_INIT_3B_WAIT: begin
                    if (lcd_done) begin
                        delay_count <= WAIT_1MS - 1;
                        state       <= ST_INIT_3B_DELAY;
                    end
                end

                ST_INIT_3B_DELAY: begin
                    if (delay_count == 0)
                        state <= ST_INIT_3C_SEND;
                    else
                        delay_count <= delay_count - 1'b1;
                end

                ST_INIT_3C_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b0;
                        lcd_nibble_only <= 1'b1;
                        lcd_data        <= 8'h30;
                        lcd_start       <= 1'b1;
                        state           <= ST_INIT_3C_WAIT;
                    end
                end

                ST_INIT_3C_WAIT: begin
                    if (lcd_done) begin
                        delay_count <= WAIT_1MS - 1;
                        state       <= ST_INIT_3C_DELAY;
                    end
                end

                ST_INIT_3C_DELAY: begin
                    if (delay_count == 0)
                        state <= ST_INIT_2_SEND;
                    else
                        delay_count <= delay_count - 1'b1;
                end

                ST_INIT_2_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b0;
                        lcd_nibble_only <= 1'b1;
                        lcd_data        <= 8'h20;
                        lcd_start       <= 1'b1;
                        state           <= ST_INIT_2_WAIT;
                    end
                end

                ST_INIT_2_WAIT: begin
                    if (lcd_done)
                        state <= ST_CMD_28_SEND;
                end

                ST_CMD_28_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b0;
                        lcd_nibble_only <= 1'b0;
                        lcd_data        <= 8'h28;
                        lcd_start       <= 1'b1;
                        state           <= ST_CMD_28_WAIT;
                    end
                end

                ST_CMD_28_WAIT: begin
                    if (lcd_done)
                        state <= ST_CMD_08_SEND;
                end

                ST_CMD_08_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b0;
                        lcd_nibble_only <= 1'b0;
                        lcd_data        <= 8'h08;
                        lcd_start       <= 1'b1;
                        state           <= ST_CMD_08_WAIT;
                    end
                end

                ST_CMD_08_WAIT: begin
                    if (lcd_done)
                        state <= ST_CMD_01_SEND;
                end

                ST_CMD_01_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b0;
                        lcd_nibble_only <= 1'b0;
                        lcd_data        <= 8'h01;
                        lcd_start       <= 1'b1;
                        state           <= ST_CMD_01_WAIT;
                    end
                end

                ST_CMD_01_WAIT: begin
                    if (lcd_done) begin
                        delay_count <= WAIT_2MS - 1;
                        state       <= ST_CMD_01_DELAY;
                    end
                end

                ST_CMD_01_DELAY: begin
                    if (delay_count == 0)
                        state <= ST_CMD_06_SEND;
                    else
                        delay_count <= delay_count - 1'b1;
                end

                ST_CMD_06_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b0;
                        lcd_nibble_only <= 1'b0;
                        lcd_data        <= 8'h06;
                        lcd_start       <= 1'b1;
                        state           <= ST_CMD_06_WAIT;
                    end
                end

                ST_CMD_06_WAIT: begin
                    if (lcd_done)
                        state <= ST_CMD_0C_SEND;
                end

                ST_CMD_0C_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b0;
                        lcd_nibble_only <= 1'b0;
                        lcd_data        <= 8'h0C;
                        lcd_start       <= 1'b1;
                        state           <= ST_CMD_0C_WAIT;
                    end
                end

                ST_CMD_0C_WAIT: begin
                    if (lcd_done)
                        state <= ST_READY;
                end

                ST_READY: begin
                    if (handled_toggle != data_toggle) begin
                        handled_toggle <= data_toggle;
                        state          <= ST_LINE1_ADDR_SEND;
                    end
                end

                ST_LINE1_ADDR_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b0;
                        lcd_nibble_only <= 1'b0;
                        lcd_data        <= 8'h80;
                        lcd_start       <= 1'b1;
                        state           <= ST_LINE1_ADDR_WAIT;
                    end
                end

                ST_LINE1_ADDR_WAIT: begin
                    if (lcd_done) begin
                        char_index <= 5'd0;
                        state      <= ST_LINE1_CHAR_SEND;
                    end
                end

                ST_LINE1_CHAR_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b1;
                        lcd_nibble_only <= 1'b0;
                        lcd_data        <= line1_char(char_index);
                        lcd_start       <= 1'b1;
                        state           <= ST_LINE1_CHAR_WAIT;
                    end
                end

                ST_LINE1_CHAR_WAIT: begin
                    if (lcd_done) begin
                        if (char_index == 5'd15)
                            state <= ST_LINE2_ADDR_SEND;
                        else begin
                            char_index <= char_index + 1'b1;
                            state      <= ST_LINE1_CHAR_SEND;
                        end
                    end
                end

                ST_LINE2_ADDR_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b0;
                        lcd_nibble_only <= 1'b0;
                        lcd_data        <= 8'hC0;
                        lcd_start       <= 1'b1;
                        state           <= ST_LINE2_ADDR_WAIT;
                    end
                end

                ST_LINE2_ADDR_WAIT: begin
                    if (lcd_done) begin
                        char_index <= 5'd0;
                        state      <= ST_LINE2_CHAR_SEND;
                    end
                end

                ST_LINE2_CHAR_SEND: begin
                    if (!lcd_busy) begin
                        lcd_rs          <= 1'b1;
                        lcd_nibble_only <= 1'b0;
                        lcd_data        <= line2_char(char_index);
                        lcd_start       <= 1'b1;
                        state           <= ST_LINE2_CHAR_WAIT;
                    end
                end

                ST_LINE2_CHAR_WAIT: begin
                    if (lcd_done) begin
                        if (char_index == 5'd15)
                            state <= ST_READY;
                        else begin
                            char_index <= char_index + 1'b1;
                            state      <= ST_LINE2_CHAR_SEND;
                        end
                    end
                end

                default: begin
                    state       <= ST_PWR_DELAY;
                    delay_count <= WAIT_50MS - 1;
                end
            endcase
        end
    end

endmodule