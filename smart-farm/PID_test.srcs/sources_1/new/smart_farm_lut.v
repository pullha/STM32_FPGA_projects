`timescale 1ns / 1ps

module smart_farm_lut #(
    parameter integer CLK_FREQ = 100_000_000
)(
    input  wire       clk,
    input  wire       reset,

    // HC-SR04
    input  wire       echo_in,
    output wire       trig_out,

    // XADC water-level input - VAUX7 only
    input  wire       vauxp7,
    input  wire       vauxn7,

    // Basys3 built-in 7-segment
    output reg  [3:0] an,
    output reg  [6:0] seg,

    // Basys3 internal LEDs
    output wire [11:0] led0,

    // Water pump control
    output reg        pump_on
);

    // XADC single channel
    wire [11:0] water_adc;

    XXadc_dual_u u_xadc_ch7 (
        .dclk_in (clk),
        .vauxp7  (vauxp7),
        .vauxn7  (vauxn7),
        .adc_ch7 (water_adc)
    );

    // Water level ADC -> 7-segment display
    reg [23:0] water_filter_reg = 24'd0;

    always @(posedge clk) begin
        water_filter_reg <= water_filter_reg - (water_filter_reg >> 10) + water_adc;
    end

    wire [11:0] stable_water_adc = water_filter_reg[21:10];

    localparam [11:0] WATER_THRES_0 = 12'd100;
    localparam [11:0] WATER_THRES_1 = 12'd1500;
    localparam [11:0] WATER_THRES_2 = 12'd1700;
    localparam [11:0] WATER_THRES_3 = 12'd1850;

    reg [3:0] water_level = 4'd0;

    always @(*) begin
        if      (stable_water_adc < WATER_THRES_0) water_level = 4'd0;
        else if (stable_water_adc < WATER_THRES_1) water_level = 4'd1;
        else if (stable_water_adc < WATER_THRES_2) water_level = 4'd2;
        else if (stable_water_adc < WATER_THRES_3) water_level = 4'd3;
        else                                       water_level = 4'd4;
    end

    reg [19:0] fnd_refresh_cnt = 20'd0;
    wire [1:0] fnd_digit_sel;

    always @(posedge clk) begin
        fnd_refresh_cnt <= fnd_refresh_cnt + 1'b1;
    end

    assign fnd_digit_sel = fnd_refresh_cnt[19:18];

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

        case (fnd_digit_sel)
            2'd0: begin
                an  = 4'b1110;
                seg = seg_digit(water_level);
            end

            2'd1: begin
                an  = 4'b1101;
                seg = SEG_BLANK;
            end

            default: begin
                an  = 4'b1111;
                seg = SEG_BLANK;
            end
        endcase
    end

    // Water pump hysteresis: on at level 0~2, off at level 4, hold at level 3.
    always @(posedge clk) begin
        if (reset) begin
            pump_on <= 1'b0;
        end else if (water_level <= 4'd2) begin
            pump_on <= 1'b1;
        end else if (water_level >= 4'd4) begin
            pump_on <= 1'b0;
        end
    end

    // 100ms automatic ultrasonic trigger
    reg [23:0] auto_trig_counter = 24'd0;
    reg        start_measure = 1'b0;

    always @(posedge clk) begin
        if (reset) begin
            auto_trig_counter <= 24'd0;
            start_measure     <= 1'b0;
        end else if (auto_trig_counter >= (CLK_FREQ / 10) - 1) begin
            auto_trig_counter <= 24'd0;
            start_measure     <= 1'b1;
        end else begin
            auto_trig_counter <= auto_trig_counter + 1'b1;
            start_measure     <= 1'b0;
        end
    end

    wire [9:0] distance_cm;
    wire       distance_valid;
    wire       distance_timeout;
    wire       distance_busy;

    ultrasonic_distance_meter_once #(
        .CLK_FREQ_HZ (CLK_FREQ)
    ) u_ultra_once (
        .clk            (clk),
        .reset          (reset),
        .start          (start_measure),
        .echo           (echo_in),
        .trig           (trig_out),
        .distance_cm    (distance_cm),
        .distance_valid (distance_valid),
        .busy           (distance_busy),
        .timeout_error  (distance_timeout)
    );

    // Plant length in 0.1cm units:
    // plant_length_x10 = 10.0cm - water_level_x10 - distance_x10.
    reg  [13:0] water_level_x10;
    wire [13:0] distance_cm_ext;
    wire [13:0] distance_x10;
    wire [13:0] occupied_height_x10;
    wire [9:0]  next_plant_length_x10;
    reg  [9:0]  plant_length_x10 = 10'd0;
    reg  [9:0]  plant_length_next_reg = 10'd0;
    reg         plant_length_update_pending = 1'b0;

    always @(*) begin
        case (water_level)
            4'd0: water_level_x10 = 14'd0;
            4'd1: water_level_x10 = 14'd10;
            4'd2: water_level_x10 = 14'd20;
            4'd3: water_level_x10 = 14'd30;
            default: water_level_x10 = 14'd40;
        endcase
    end

    assign distance_cm_ext = {4'd0, distance_cm};
    assign distance_x10    = (distance_cm_ext << 3) + (distance_cm_ext << 1);
    assign occupied_height_x10 = water_level_x10 + distance_x10;
    assign next_plant_length_x10 =
        (occupied_height_x10 < 14'd100) ? (10'd100 - occupied_height_x10[9:0]) : 10'd0;

    reg has_measured = 1'b0;

    reg  distance_valid_d = 1'b0;
    wire distance_valid_pe;

    always @(posedge clk) begin
        if (reset)
            distance_valid_d <= 1'b0;
        else
            distance_valid_d <= distance_valid;
    end

    assign distance_valid_pe = distance_valid && !distance_valid_d;

    localparam STATE_IDLE  = 2'd0;
    localparam STATE_SEQ   = 2'd1;
    localparam STATE_BLINK = 2'd2;

    reg [1:0]  anim_state = STATE_IDLE;
    reg [23:0] seq_timer = 24'd0;
    reg [3:0]  seq_index = 4'd0;
    reg [29:0] blink_total_timer = 30'd0;
    reg [25:0] blink_toggle_timer = 26'd0;
    reg        blink_out = 1'b0;
    reg [11:0] led_reg = 12'b0000_0000_0000;

    function [11:0] plant_length_led_bar;
        input [9:0] length_x10;
        begin
            if      (length_x10 <= 10'd15) plant_length_led_bar = 12'b0000_0000_1111;
            else if (length_x10 <= 10'd18) plant_length_led_bar = 12'b0000_0001_1111;
            else if (length_x10 <= 10'd22) plant_length_led_bar = 12'b0000_0011_1111;
            else if (length_x10 <= 10'd26) plant_length_led_bar = 12'b0000_0111_1111;
            else if (length_x10 <= 10'd29) plant_length_led_bar = 12'b0000_1111_1111;
            else                            plant_length_led_bar = 12'b1111_1111_1111;
        end
    endfunction

    always @(posedge clk) begin
        if (reset) begin
            plant_length_x10   <= 10'd0;
            plant_length_next_reg <= 10'd0;
            plant_length_update_pending <= 1'b0;
            has_measured       <= 1'b0;
            anim_state         <= STATE_IDLE;
            seq_timer          <= 24'd0;
            seq_index          <= 4'd0;
            blink_total_timer  <= 30'd0;
            blink_toggle_timer <= 26'd0;
            blink_out          <= 1'b0;
        end else begin
            case (anim_state)
                STATE_IDLE: begin
                end

                STATE_SEQ: begin
                    if (seq_timer >= (CLK_FREQ / 10) - 1) begin
                        seq_timer <= 24'd0;
                        if (seq_index == 4'd12) begin
                            anim_state         <= STATE_BLINK;
                            blink_total_timer  <= 30'd0;
                            blink_toggle_timer <= 26'd0;
                            blink_out          <= 1'b1;
                        end else begin
                            seq_index <= seq_index + 1'b1;
                        end
                    end else begin
                        seq_timer <= seq_timer + 1'b1;
                    end
                end

                STATE_BLINK: begin
                    if (blink_total_timer >= (CLK_FREQ * 6) - 1) begin
                        anim_state <= STATE_IDLE;
                    end else begin
                        blink_total_timer <= blink_total_timer + 1'b1;

                        if (blink_toggle_timer >= (CLK_FREQ / 2) - 1) begin
                            blink_toggle_timer <= 26'd0;
                            blink_out          <= ~blink_out;
                        end else begin
                            blink_toggle_timer <= blink_toggle_timer + 1'b1;
                        end
                    end
                end

                default: begin
                    anim_state <= STATE_IDLE;
                end
            endcase

            if (plant_length_update_pending) begin
                plant_length_update_pending <= 1'b0;
                plant_length_x10 <= plant_length_next_reg;
                has_measured     <= 1'b1;

                if (plant_length_next_reg >= 10'd60) begin
                    if (anim_state == STATE_IDLE) begin
                        anim_state <= STATE_SEQ;
                        seq_timer  <= 24'd0;
                        seq_index  <= 4'd0;
                    end
                end else begin
                    anim_state <= STATE_IDLE;
                end
            end else if (distance_valid_pe) begin
                plant_length_next_reg <= next_plant_length_x10;
                plant_length_update_pending <= 1'b1;
            end
        end
    end

    always @(*) begin
        case (anim_state)
            STATE_IDLE: begin
                if (!has_measured) begin
                    led_reg = 12'b0000_0000_0000;
                end else begin
                    led_reg = plant_length_led_bar(plant_length_x10);
                end
            end

            STATE_SEQ: begin
                case (seq_index)
                    4'd0:  led_reg = 12'b0000_0000_0000;
                    4'd1:  led_reg = 12'b0000_0000_0001;
                    4'd2:  led_reg = 12'b0000_0000_0011;
                    4'd3:  led_reg = 12'b0000_0000_0111;
                    4'd4:  led_reg = 12'b0000_0000_1111;
                    4'd5:  led_reg = 12'b0000_0001_1111;
                    4'd6:  led_reg = 12'b0000_0011_1111;
                    4'd7:  led_reg = 12'b0000_0111_1111;
                    4'd8:  led_reg = 12'b0000_1111_1111;
                    4'd9:  led_reg = 12'b0001_1111_1111;
                    4'd10: led_reg = 12'b0011_1111_1111;
                    4'd11: led_reg = 12'b0111_1111_1111;
                    4'd12: led_reg = 12'b1111_1111_1111;
                    default: led_reg = 12'b0000_0000_0000;
                endcase
            end

            STATE_BLINK: begin
                led_reg = blink_out ? 12'b1111_1111_1111 : 12'b0000_0000_0000;
            end

            default: begin
                led_reg = 12'b0000_0000_0000;
            end
        endcase
    end

    assign led0 = led_reg;

endmodule
