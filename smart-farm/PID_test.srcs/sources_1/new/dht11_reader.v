`timescale 1ns / 1ps

// ============================================================
// DHT11 Reader
//
// DHT11과 1-wire 방식으로 통신해서 습도/온도를 읽는다.
// data_valid = 1이면 humidity, temperature 값이 정상이다.
// error = 1이면 응답 timeout 또는 checksum 오류이다.
// ============================================================
module dht11_reader #(
    parameter CLK_HZ               = 100000000,
    parameter POWERUP_DELAY_US     = 1000000,
    parameter READ_INTERVAL_US     = 2000000,
    parameter START_LOW_US         = 20000,
    parameter RESP_TIMEOUT_US      = 200,
    parameter BIT_LOW_TIMEOUT_US   = 120,
    parameter BIT_HIGH_TIMEOUT_US  = 120,
    parameter BIT_ONE_THRESHOLD_US = 40
)(
    input  wire       clk,
    input  wire       rst,
    inout  wire       dht,
    output reg  [7:0] humidity,
    output reg  [7:0] temperature,
    output reg        data_valid,
    output reg        error
);

    // 1us마다 한 번씩 tick을 발생시킨다.
    wire tick_1us;

    tick_1us_gen #(
        .CLK_HZ(CLK_HZ)
    ) u_tick_1us_gen (
        .clk  (clk),
        .rst  (rst),
        .tick (tick_1us)
    );

    // DHT11 DATA는 open-drain처럼 사용한다.
    // FPGA가 0을 보낼 때만 drive_low = 1로 LOW 출력한다.
    // 그 외에는 high-Z로 풀어 DHT11이 라인을 구동하게 한다.
    reg drive_low;
    assign dht = drive_low ? 1'b0 : 1'bz;

    wire dht_raw = dht;

    // 외부 DHT11 입력을 clk에 동기화한다.
    reg dht_meta;
    reg dht_sync;

    always @(posedge clk) begin
        if (rst) begin
            dht_meta <= 1'b1;
            dht_sync <= 1'b1;
        end else begin
            dht_meta <= dht_raw;
            dht_sync <= dht_meta;
        end
    end

    // DHT11 통신 상태 머신
    localparam S_POWERUP        = 4'd0;
    localparam S_IDLE           = 4'd1;
    localparam S_START_LOW      = 4'd2;
    localparam S_WAIT_RESP_LOW  = 4'd3;
    localparam S_WAIT_RESP_HIGH = 4'd4;
    localparam S_WAIT_RESP_END  = 4'd5;
    localparam S_BIT_LOW        = 4'd6;
    localparam S_BIT_HIGH       = 4'd7;
    localparam S_DONE           = 4'd8;
    localparam S_FAIL           = 4'd9;

    reg [3:0]  state;
    reg [31:0] us_count;
    reg [5:0]  bit_count;
    reg [39:0] shift_data;
    reg [39:0] frame;

    // DHT11 checksum = 앞의 4바이트 합의 하위 8비트
    wire [7:0] checksum_calc;
    assign checksum_calc = frame[39:32] + frame[31:24] + frame[23:16] + frame[15:8];

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_POWERUP;
            us_count    <= 32'd0;
            bit_count   <= 6'd0;
            shift_data  <= 40'd0;
            frame       <= 40'd0;
            drive_low   <= 1'b0;
            humidity    <= 8'd0;
            temperature <= 8'd0;
            data_valid  <= 1'b0;
            error       <= 1'b0;
        end else begin
            if (tick_1us) begin
                case (state)
                    S_POWERUP: begin
                        drive_low <= 1'b0;
                        error <= 1'b0;

                        if (us_count >= POWERUP_DELAY_US - 1) begin
                            us_count <= 32'd0;
                            state <= S_START_LOW;
                        end else begin
                            us_count <= us_count + 1'b1;
                        end
                    end

                    S_IDLE: begin
                        drive_low <= 1'b0;

                        if (us_count >= READ_INTERVAL_US - 1) begin
                            us_count   <= 32'd0;
                            bit_count  <= 6'd0;
                            shift_data <= 40'd0;
                            frame      <= 40'd0;
                            error      <= 1'b0;
                            state      <= S_START_LOW;
                        end else begin
                            us_count <= us_count + 1'b1;
                        end
                    end

                    S_START_LOW: begin
                        // FPGA가 DHT11에 시작 신호를 보낸다.
                        drive_low <= 1'b1;

                        if (us_count >= START_LOW_US - 1) begin
                            drive_low <= 1'b0;
                            us_count  <= 32'd0;
                            state     <= S_WAIT_RESP_LOW;
                        end else begin
                            us_count <= us_count + 1'b1;
                        end
                    end

                    S_WAIT_RESP_LOW: begin
                        drive_low <= 1'b0;

                        if (dht_sync == 1'b0) begin
                            us_count <= 32'd0;
                            state <= S_WAIT_RESP_HIGH;
                        end else if (us_count >= RESP_TIMEOUT_US) begin
                            us_count <= 32'd0;
                            state <= S_FAIL;
                        end else begin
                            us_count <= us_count + 1'b1;
                        end
                    end

                    S_WAIT_RESP_HIGH: begin
                        if (dht_sync == 1'b1) begin
                            us_count <= 32'd0;
                            state <= S_WAIT_RESP_END;
                        end else if (us_count >= RESP_TIMEOUT_US) begin
                            us_count <= 32'd0;
                            state <= S_FAIL;
                        end else begin
                            us_count <= us_count + 1'b1;
                        end
                    end

                    S_WAIT_RESP_END: begin
                        if (dht_sync == 1'b0) begin
                            us_count <= 32'd0;
                            state <= S_BIT_LOW;
                        end else if (us_count >= RESP_TIMEOUT_US) begin
                            us_count <= 32'd0;
                            state <= S_FAIL;
                        end else begin
                            us_count <= us_count + 1'b1;
                        end
                    end

                    S_BIT_LOW: begin
                        if (dht_sync == 1'b1) begin
                            us_count <= 32'd0;
                            state <= S_BIT_HIGH;
                        end else if (us_count >= BIT_LOW_TIMEOUT_US) begin
                            us_count <= 32'd0;
                            state <= S_FAIL;
                        end else begin
                            us_count <= us_count + 1'b1;
                        end
                    end

                    S_BIT_HIGH: begin
                        if (dht_sync == 1'b0) begin
                            // HIGH 시간이 길면 1, 짧으면 0으로 판단한다.
                            if (bit_count == 6'd39) begin
                                frame <= {shift_data[38:0], (us_count >= BIT_ONE_THRESHOLD_US)};
                                us_count <= 32'd0;
                                state <= S_DONE;
                            end else begin
                                shift_data <= {shift_data[38:0], (us_count >= BIT_ONE_THRESHOLD_US)};
                                bit_count <= bit_count + 1'b1;
                                us_count <= 32'd0;
                                state <= S_BIT_LOW;
                            end
                        end else if (us_count >= BIT_HIGH_TIMEOUT_US) begin
                            us_count <= 32'd0;
                            state <= S_FAIL;
                        end else begin
                            us_count <= us_count + 1'b1;
                        end
                    end

                    S_DONE: begin
                        drive_low <= 1'b0;
                        us_count <= 32'd0;

                        if (checksum_calc == frame[7:0]) begin
                            humidity    <= frame[39:32];
                            temperature <= frame[23:16];
                            data_valid  <= 1'b1;
                            error       <= 1'b0;
                        end else begin
                            data_valid <= 1'b0;
                            error <= 1'b1;
                        end

                        state <= S_IDLE;
                    end

                    S_FAIL: begin
                        drive_low <= 1'b0;
                        data_valid <= 1'b0;
                        error <= 1'b1;
                        us_count <= 32'd0;
                        state <= S_IDLE;
                    end

                    default: begin
                        drive_low <= 1'b0;
                        us_count <= 32'd0;
                        state <= S_POWERUP;
                    end
                endcase
            end
        end
    end

endmodule
