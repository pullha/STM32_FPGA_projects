`timescale 1ns / 1ps

// ============================================================
// Flame UART Warning
//
// flame = 0이면 불꽃 감지로 판단한다.
// 불꽃 감지 중 부저를 울리고 UART로 "FIRE WARNING"을 반복 송신한다.
// ============================================================
module flame_uart_warning #(
    parameter integer CLK_FREQ = 100000000,
    parameter integer BAUD_RATE = 9600,

    // 불꽃 감지 중 FIRE WARNING을 1초마다 반복 송신
    parameter integer FIRE_REPEAT_CYCLES = 100000000,

    // 부저 주파수 설정용
    parameter integer BUZZ_HALF_PERIOD = 20833
)(
    input  wire clk,
    input  wire rst,
    input  wire flame,
    output wire sound,
    output wire RsTx
);

    // 외부 flame 입력 동기화
    reg flame_meta;
    reg flame_sync;

    always @(posedge clk) begin
        if (rst) begin
            flame_meta <= 1'b1;
            flame_sync <= 1'b1;
        end else begin
            flame_meta <= flame;
            flame_sync <= flame_meta;
        end
    end

    // ========================================================
    // Flame input noise filter
    // flame_sync가 30ms 동안 연속 LOW일 때만 불꽃 감지로 인정
    // 100MHz 기준: 100,000,000 × 0.03초 = 3,000,000클럭
    // ========================================================
    localparam integer FLAME_FILTER_CYCLES = 3_000_000;

    reg [31:0] flame_low_count;
    reg        fire_detected;

    always @(posedge clk) begin
        if (rst) begin
            flame_low_count <= 32'd0;
            fire_detected   <= 1'b0;
        end else begin
            if (flame_sync == 1'b0) begin
                if (flame_low_count < FLAME_FILTER_CYCLES - 1) begin
                    flame_low_count <= flame_low_count + 1'b1;
                end else begin
                    flame_low_count <= flame_low_count;
                    fire_detected   <= 1'b1;
                end
            end else begin
                flame_low_count <= 32'd0;
                fire_detected   <= 1'b0;
            end
        end
    end

    // ========================================================
    // Buzzer
    // fire_detected = 1이면 일정 주기로 sound를 toggle한다.
    // ========================================================
    reg [31:0] buzz_count;
    reg        buzz_reg;

    always @(posedge clk) begin
        if (rst) begin
            buzz_count <= 32'd0;
            buzz_reg   <= 1'b0;
        end else if (fire_detected) begin
            if (buzz_count >= BUZZ_HALF_PERIOD - 1) begin
                buzz_count <= 32'd0;
                buzz_reg   <= ~buzz_reg;
            end else begin
                buzz_count <= buzz_count + 1'b1;
            end
        end else begin
            buzz_count <= 32'd0;
            buzz_reg   <= 1'b0;
        end
    end

    assign sound = buzz_reg;

    // ========================================================
    // UART TX
    // ========================================================
    reg  [7:0] tx_data;
    reg        tx_start;
    wire       tx_busy;
    wire       tx_done;

    uart_tx_byte #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_uart_tx_byte (
        .clk    (clk),
        .rst    (rst),
        .data   (tx_data),
        .start  (tx_start),
        .tx     (RsTx),
        .busy   (tx_busy),
        .done   (tx_done)
    );

    // "FIRE WARNING\r\n"
    localparam [3:0] MSG_LAST_INDEX = 4'd13;

    function [7:0] msg_byte;
        input [3:0] index;
        begin
            case (index)
                4'd0:  msg_byte = 8'h46; // F
                4'd1:  msg_byte = 8'h49; // I
                4'd2:  msg_byte = 8'h52; // R
                4'd3:  msg_byte = 8'h45; // E
                4'd4:  msg_byte = 8'h20; // space
                4'd5:  msg_byte = 8'h57; // W
                4'd6:  msg_byte = 8'h41; // A
                4'd7:  msg_byte = 8'h52; // R
                4'd8:  msg_byte = 8'h4E; // N
                4'd9:  msg_byte = 8'h49; // I
                4'd10: msg_byte = 8'h4E; // N
                4'd11: msg_byte = 8'h47; // G
                4'd12: msg_byte = 8'h0D; // carriage return
                4'd13: msg_byte = 8'h0A; // line feed
                default: msg_byte = 8'h00;
            endcase
        end
    endfunction

    localparam M_IDLE        = 2'd0;
    localparam M_SEND_BYTE   = 2'd1;
    localparam M_WAIT_DONE   = 2'd2;
    localparam M_WAIT_REPEAT = 2'd3;

    reg [1:0]  msg_state;
    reg [3:0]  msg_index;
    reg [31:0] repeat_count;

    always @(posedge clk) begin
        if (rst) begin
            msg_state    <= M_IDLE;
            msg_index    <= 4'd0;
            repeat_count <= 32'd0;
            tx_data      <= 8'd0;
            tx_start     <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            case (msg_state)
                M_IDLE: begin
                    msg_index    <= 4'd0;
                    repeat_count <= 32'd0;

                    if (fire_detected)
                        msg_state <= M_SEND_BYTE;
                end

                M_SEND_BYTE: begin
                    if (!fire_detected) begin
                        msg_state <= M_IDLE;
                        msg_index <= 4'd0;
                    end else if (!tx_busy) begin
                        tx_data   <= msg_byte(msg_index);
                        tx_start  <= 1'b1;
                        msg_state <= M_WAIT_DONE;
                    end
                end

                M_WAIT_DONE: begin
                    if (!fire_detected) begin
                        msg_state <= M_IDLE;
                        msg_index <= 4'd0;
                    end else if (tx_done) begin
                        if (msg_index == MSG_LAST_INDEX) begin
                            msg_index    <= 4'd0;
                            repeat_count <= 32'd0;
                            msg_state    <= M_WAIT_REPEAT;
                        end else begin
                            msg_index <= msg_index + 1'b1;
                            msg_state <= M_SEND_BYTE;
                        end
                    end
                end

                M_WAIT_REPEAT: begin
                    if (!fire_detected) begin
                        msg_state    <= M_IDLE;
                        repeat_count <= 32'd0;
                    end else if (repeat_count >= FIRE_REPEAT_CYCLES - 1) begin
                        repeat_count <= 32'd0;
                        msg_index    <= 4'd0;
                        msg_state    <= M_SEND_BYTE;
                    end else begin
                        repeat_count <= repeat_count + 1'b1;
                    end
                end

                default: begin
                    msg_state <= M_IDLE;
                end
            endcase
        end
    end

endmodule