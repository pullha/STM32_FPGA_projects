`timescale 1ns / 1ps

module i2c_master_write #(
    parameter integer CLK_HZ  = 100_000_000,
    parameter integer I2C_HZ  = 100_000,
    parameter [6:0]   I2C_ADDR = 7'h27
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    input  wire [7:0] data_in,
    output reg        busy,
    output reg        done,
    output reg        ack_error,
    inout  wire       i2c_scl,
    inout  wire       i2c_sda
);

    localparam integer HALF_PERIOD = CLK_HZ / (I2C_HZ * 2);

    reg [15:0] div_count;
    reg        scl_drive_low;
    reg        sda_drive_low;

    assign i2c_scl = scl_drive_low ? 1'b0 : 1'bz;
    assign i2c_sda = sda_drive_low ? 1'b0 : 1'bz;

    wire sda_in = i2c_sda;

    localparam [4:0]
        ST_IDLE            = 5'd0,
        ST_START_HOLD      = 5'd1,
        ST_ADDR_SETUP      = 5'd2,
        ST_ADDR_HIGH       = 5'd3,
        ST_ADDR_LOW        = 5'd4,
        ST_ADDR_ACK_SETUP  = 5'd5,
        ST_ADDR_ACK_HIGH   = 5'd6,
        ST_DATA_SETUP      = 5'd7,
        ST_DATA_HIGH       = 5'd8,
        ST_DATA_LOW        = 5'd9,
        ST_DATA_ACK_SETUP  = 5'd10,
        ST_DATA_ACK_HIGH   = 5'd11,
        ST_STOP_LOW        = 5'd12,
        ST_STOP_HIGH       = 5'd13,
        ST_STOP_RELEASE    = 5'd14;

    reg [4:0] state;
    reg [7:0] tx_byte;
    reg [7:0] data_latched;
    reg [2:0] bit_index;

    wire half_tick = (div_count == HALF_PERIOD - 1);

    always @(posedge clk) begin
        if (rst) begin
            div_count     <= 16'd0;
            scl_drive_low <= 1'b0;
            sda_drive_low <= 1'b0;
            busy          <= 1'b0;
            done          <= 1'b0;
            ack_error     <= 1'b0;
            state         <= ST_IDLE;
            tx_byte       <= 8'd0;
            data_latched  <= 8'd0;
            bit_index     <= 3'd7;
        end else begin
            done <= 1'b0;

            if (!busy) begin
                div_count <= 16'd0;
            end else if (half_tick) begin
                div_count <= 16'd0;
            end else begin
                div_count <= div_count + 1'b1;
            end

            case (state)
                ST_IDLE: begin
                    scl_drive_low <= 1'b0;
                    sda_drive_low <= 1'b0;
                    busy          <= 1'b0;

                    if (start) begin
                        busy          <= 1'b1;
                        ack_error     <= 1'b0;
                        data_latched  <= data_in;
                        tx_byte       <= {I2C_ADDR, 1'b0};
                        bit_index     <= 3'd7;
                        sda_drive_low <= 1'b1;
                        state         <= ST_START_HOLD;
                    end
                end

                ST_START_HOLD: begin
                    if (half_tick) begin
                        scl_drive_low <= 1'b1;
                        state         <= ST_ADDR_SETUP;
                    end
                end

                ST_ADDR_SETUP: begin
                    if (half_tick) begin
                        sda_drive_low <= ~tx_byte[bit_index];
                        state         <= ST_ADDR_HIGH;
                    end
                end

                ST_ADDR_HIGH: begin
                    if (half_tick) begin
                        scl_drive_low <= 1'b0;
                        state         <= ST_ADDR_LOW;
                    end
                end

                ST_ADDR_LOW: begin
                    if (half_tick) begin
                        scl_drive_low <= 1'b1;
                        if (bit_index == 3'd0) begin
                            sda_drive_low <= 1'b0;
                            state         <= ST_ADDR_ACK_SETUP;
                        end else begin
                            bit_index <= bit_index - 1'b1;
                            state     <= ST_ADDR_SETUP;
                        end
                    end
                end

                ST_ADDR_ACK_SETUP: begin
                    if (half_tick) begin
                        scl_drive_low <= 1'b0;
                        state         <= ST_ADDR_ACK_HIGH;
                    end
                end

                ST_ADDR_ACK_HIGH: begin
                    if (half_tick) begin
                        if (sda_in)
                            ack_error <= 1'b1;
                        scl_drive_low <= 1'b1;
                        tx_byte       <= data_latched;
                        bit_index     <= 3'd7;
                        state         <= ST_DATA_SETUP;
                    end
                end

                ST_DATA_SETUP: begin
                    if (half_tick) begin
                        sda_drive_low <= ~tx_byte[bit_index];
                        state         <= ST_DATA_HIGH;
                    end
                end

                ST_DATA_HIGH: begin
                    if (half_tick) begin
                        scl_drive_low <= 1'b0;
                        state         <= ST_DATA_LOW;
                    end
                end

                ST_DATA_LOW: begin
                    if (half_tick) begin
                        scl_drive_low <= 1'b1;
                        if (bit_index == 3'd0) begin
                            sda_drive_low <= 1'b0;
                            state         <= ST_DATA_ACK_SETUP;
                        end else begin
                            bit_index <= bit_index - 1'b1;
                            state     <= ST_DATA_SETUP;
                        end
                    end
                end

                ST_DATA_ACK_SETUP: begin
                    if (half_tick) begin
                        scl_drive_low <= 1'b0;
                        state         <= ST_DATA_ACK_HIGH;
                    end
                end

                ST_DATA_ACK_HIGH: begin
                    if (half_tick) begin
                        if (sda_in)
                            ack_error <= 1'b1;
                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b1;
                        state         <= ST_STOP_LOW;
                    end
                end

                ST_STOP_LOW: begin
                    if (half_tick) begin
                        scl_drive_low <= 1'b0;
                        state         <= ST_STOP_HIGH;
                    end
                end

                ST_STOP_HIGH: begin
                    if (half_tick) begin
                        sda_drive_low <= 1'b0;
                        state         <= ST_STOP_RELEASE;
                    end
                end

                ST_STOP_RELEASE: begin
                    if (half_tick) begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end
                end

                default: begin
                    scl_drive_low <= 1'b0;
                    sda_drive_low <= 1'b0;
                    busy          <= 1'b0;
                    ack_error     <= 1'b1;
                    state         <= ST_IDLE;
                end
            endcase
        end
    end

endmodule