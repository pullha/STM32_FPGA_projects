`timescale 1ns / 1ps

module all_module_testbench;

    reg clk;
    integer errors;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    task step_cycles;
        input integer count;
        integer i;
        begin
            for (i = 0; i < count; i = i + 1)
                @(posedge clk);
            #1;
        end
    endtask

    task fail_msg;
        input [1023:0] msg;
        begin
            $display("FAIL: %0s", msg);
            errors = errors + 1;
        end
    endtask

    task check_bit;
        input actual;
        input expected;
        input [1023:0] msg;
        begin
            if (actual !== expected) begin
                $display("FAIL: %0s expected=%0b got=%0b", msg, expected, actual);
                errors = errors + 1;
            end
        end
    endtask

    task check_u8;
        input [7:0] actual;
        input [7:0] expected;
        input [1023:0] msg;
        begin
            if (actual !== expected) begin
                $display("FAIL: %0s expected=%0d got=%0d", msg, expected, actual);
                errors = errors + 1;
            end
        end
    endtask

    task check_u10_range;
        input [9:0] actual;
        input [9:0] min_value;
        input [9:0] max_value;
        input [1023:0] msg;
        begin
            if ((actual < min_value) || (actual > max_value)) begin
                $display("FAIL: %0s expected_range=%0d..%0d got=%0d",
                         msg, min_value, max_value, actual);
                errors = errors + 1;
            end
        end
    endtask

    // ---------------------------------------------------------------------
    // tick_1us_gen
    // ---------------------------------------------------------------------
    reg tick_rst;
    wire tick_1us;

    tick_1us_gen #(
        .CLK_HZ(1_000_000)
    ) u_tick_1us_gen_tb (
        .clk  (clk),
        .rst  (tick_rst),
        .tick (tick_1us)
    );

    task run_tick_1us_gen_test;
        begin
            $display("TEST tick_1us_gen");
            tick_rst = 1'b1;
            step_cycles(2);
            tick_rst = 1'b0;
            step_cycles(1);
            check_bit(tick_1us, 1'b1, "tick_1us_gen should tick every cycle at 1 MHz");
        end
    endtask

    // ---------------------------------------------------------------------
    // uart_tx_byte
    // ---------------------------------------------------------------------
    reg uart_rst;
    reg [7:0] uart_data;
    reg uart_start;
    wire uart_tx;
    wire uart_busy;
    wire uart_done;

    uart_tx_byte #(
        .CLK_FREQ  (1_000_000),
        .BAUD_RATE (100_000)
    ) u_uart_tx_byte_tb (
        .clk   (clk),
        .rst   (uart_rst),
        .data  (uart_data),
        .start (uart_start),
        .tx    (uart_tx),
        .busy  (uart_busy),
        .done  (uart_done)
    );

    task run_uart_tx_byte_test;
        integer wait_count;
        begin
            $display("TEST uart_tx_byte");
            uart_rst   = 1'b1;
            uart_start = 1'b0;
            uart_data  = 8'hA5;
            step_cycles(3);
            uart_rst = 1'b0;
            step_cycles(1);

            uart_start = 1'b1;
            step_cycles(1);
            uart_start = 1'b0;

            check_bit(uart_tx, 1'b0, "uart_tx_byte start bit");
            wait_count = 0;
            while ((uart_done !== 1'b1) && (wait_count < 200)) begin
                step_cycles(1);
                wait_count = wait_count + 1;
            end
            check_bit(uart_done, 1'b1, "uart_tx_byte done pulse");
            check_bit(uart_tx, 1'b1, "uart_tx_byte idle high after byte");
        end
    endtask

    // ---------------------------------------------------------------------
    // ultrasonic_distance_meter_once
    // ---------------------------------------------------------------------
    reg ultra_reset;
    reg ultra_start;
    reg ultra_echo;
    wire ultra_trig;
    wire [9:0] ultra_distance_cm;
    wire ultra_distance_valid;
    wire ultra_busy;
    wire ultra_timeout_error;

    ultrasonic_distance_meter_once #(
        .CLK_FREQ_HZ     (1_000_000),
        .TRIG_PULSE_US   (4),
        .ECHO_TIMEOUT_US (2000)
    ) u_ultrasonic_distance_meter_once_tb (
        .clk            (clk),
        .reset          (ultra_reset),
        .start          (ultra_start),
        .echo           (ultra_echo),
        .trig           (ultra_trig),
        .distance_cm    (ultra_distance_cm),
        .distance_valid (ultra_distance_valid),
        .busy           (ultra_busy),
        .timeout_error  (ultra_timeout_error)
    );

    task run_ultrasonic_distance_meter_once_test;
        integer wait_count;
        begin
            $display("TEST ultrasonic_distance_meter_once");
            ultra_reset = 1'b1;
            ultra_start = 1'b0;
            ultra_echo  = 1'b0;
            step_cycles(3);
            ultra_reset = 1'b0;

            ultra_start = 1'b1;
            step_cycles(1);
            ultra_start = 1'b0;

            wait_count = 0;
            while ((ultra_trig !== 1'b1) && (wait_count < 50)) begin
                step_cycles(1);
                wait_count = wait_count + 1;
            end
            if (ultra_trig !== 1'b1)
                fail_msg("ultrasonic trig did not go high");

            wait_count = 0;
            while ((ultra_trig !== 1'b0) && (wait_count < 50)) begin
                step_cycles(1);
                wait_count = wait_count + 1;
            end

            step_cycles(5);
            ultra_echo = 1'b1;
            step_cycles(620);
            ultra_echo = 1'b0;

            wait_count = 0;
            while ((ultra_distance_valid !== 1'b1) && (wait_count < 1200)) begin
                step_cycles(1);
                wait_count = wait_count + 1;
            end

            check_bit(ultra_distance_valid, 1'b1, "ultrasonic distance_valid");
            check_u10_range(ultra_distance_cm, 10'd9, 10'd11, "ultrasonic distance around 10 cm");
        end
    endtask

    // ---------------------------------------------------------------------
    // l298_humidity_fan_controller
    // ---------------------------------------------------------------------
    reg fan_rst;
    reg fan_valid;
    reg [7:0] fan_humidity;
    wire fan_ena_pwm;
    wire fan_in1;
    wire fan_in2;
    wire [7:0] fan_duty_debug;

    l298_humidity_fan_controller #(
        .CLK_FREQ_HZ       (1_000_000),
        .PWM_FREQ_HZ       (1000),
        .HUM_START_PERCENT (8'd80),
        .HUM_FULL_PERCENT  (8'd100),
        .MIN_DUTY          (8'd128)
    ) u_l298_humidity_fan_controller_tb (
        .clk            (clk),
        .rst            (fan_rst),
        .valid          (fan_valid),
        .humidity       (fan_humidity),
        .fan_ena_pwm    (fan_ena_pwm),
        .fan_in1        (fan_in1),
        .fan_in2        (fan_in2),
        .fan_duty_debug (fan_duty_debug)
    );

    task run_l298_humidity_fan_controller_test;
        begin
            $display("TEST l298_humidity_fan_controller");
            fan_rst      = 1'b1;
            fan_valid    = 1'b0;
            fan_humidity = 8'd0;
            step_cycles(3);
            fan_rst   = 1'b0;
            fan_valid = 1'b1;

            fan_humidity = 8'd50;
            step_cycles(1);
            check_bit(fan_in1, 1'b0, "fan off below threshold in1");
            check_bit(fan_in2, 1'b0, "fan off below threshold in2");

            fan_humidity = 8'd80;
            step_cycles(1);
            check_bit(fan_in1, 1'b1, "fan starts at threshold in1");
            check_bit(fan_in2, 1'b0, "fan starts at threshold in2");
            if (fan_duty_debug < 8'd128)
                fail_msg("fan duty should be at least MIN_DUTY at 80 percent");

            fan_humidity = 8'd100;
            step_cycles(1);
            check_u8(fan_duty_debug, 8'hFF, "fan full duty at 100 percent");
        end
    endtask

    // ---------------------------------------------------------------------
    // l298n_heater_controller
    // ---------------------------------------------------------------------
    reg heater_rst;
    reg heater_valid;
    reg heater_error;
    reg [7:0] heater_temperature;
    wire heater_l298n_in1;

    l298n_heater_controller #(
        .CLK_HZ             (100_000),
        .SET_TEMP_C         (8'd30),
        .CUTOFF_TEMP_C      (8'd29),
        .RESTART_TEMP_C     (8'd28),
        .KP                 (16'sd80),
        .KI                 (16'sd10),
        .INTEGRAL_MIN       (32'sd0),
        .INTEGRAL_MAX       (32'sd100),
        .MAX_DUTY_PERMILLE  (250),
        .PWM_PERIOD_MS      (10),
        .PI_SAMPLE_MS       (1)
    ) u_l298n_heater_controller_tb (
        .clk         (clk),
        .rst         (heater_rst),
        .valid       (heater_valid),
        .error       (heater_error),
        .temperature (heater_temperature),
        .l298n_in1   (heater_l298n_in1)
    );

    task run_l298n_heater_controller_test;
        begin
            $display("TEST l298n_heater_controller");
            heater_rst         = 1'b1;
            heater_valid       = 1'b0;
            heater_error       = 1'b0;
            heater_temperature = 8'd30;
            step_cycles(5);
            heater_rst   = 1'b0;
            heater_valid = 1'b1;

            heater_temperature = 8'd27;
            step_cycles(20);
            check_bit(u_l298n_heater_controller_tb.heater_allowed, 1'b1,
                      "heater allowed below restart temperature");

            heater_temperature = 8'd29;
            step_cycles(130);
            check_bit(u_l298n_heater_controller_tb.heater_allowed, 1'b0,
                      "heater off at cutoff temperature");

            heater_error = 1'b1;
            step_cycles(2);
            check_bit(heater_l298n_in1, 1'b0, "heater output off on sensor error");
            heater_error = 1'b0;
        end
    endtask

    // ---------------------------------------------------------------------
    // servo_window_control
    // ---------------------------------------------------------------------
    reg servo_rst;
    reg servo_valid;
    reg [7:0] servo_temperature;
    reg servo_force_closed;
    wire servo_pwm;

    servo_window_control #(
        .CLK_HZ          (1_000_000),
        .OPEN_TEMP_C     (8'd31),
        .CLOSE_TEMP_C    (8'd30),
        .CLOSED_PULSE_US (2),
        .OPEN_PULSE_US   (4)
    ) u_servo_window_control_tb (
        .clk          (clk),
        .rst          (servo_rst),
        .valid        (servo_valid),
        .temperature  (servo_temperature),
        .force_closed (servo_force_closed),
        .servo_pwm    (servo_pwm)
    );

    task run_servo_window_control_test;
        begin
            $display("TEST servo_window_control");
            servo_rst          = 1'b1;
            servo_valid        = 1'b0;
            servo_temperature  = 8'd0;
            servo_force_closed = 1'b0;
            step_cycles(3);
            servo_rst = 1'b0;

            servo_valid       = 1'b1;
            servo_temperature = 8'd31;
            step_cycles(1);
            check_bit(u_servo_window_control_tb.window_open, 1'b1, "servo opens at high temperature");

            servo_force_closed = 1'b1;
            step_cycles(1);
            check_bit(u_servo_window_control_tb.window_open, 1'b0, "servo force close");
            servo_force_closed = 1'b0;
        end
    endtask

    // ---------------------------------------------------------------------
    // i2c_master_write
    // ---------------------------------------------------------------------
    reg i2c_master_rst;
    reg i2c_master_start;
    reg [7:0] i2c_master_data;
    wire i2c_master_busy;
    wire i2c_master_done;
    wire i2c_master_ack_error;
    tri i2c_master_scl;
    tri i2c_master_sda;
    pullup(i2c_master_scl);
    pullup(i2c_master_sda);

    i2c_master_write #(
        .CLK_HZ   (1_000_000),
        .I2C_HZ   (100_000),
        .I2C_ADDR (7'h27)
    ) u_i2c_master_write_tb (
        .clk       (clk),
        .rst       (i2c_master_rst),
        .start     (i2c_master_start),
        .data_in   (i2c_master_data),
        .busy      (i2c_master_busy),
        .done      (i2c_master_done),
        .ack_error (i2c_master_ack_error),
        .i2c_scl   (i2c_master_scl),
        .i2c_sda   (i2c_master_sda)
    );

    task run_i2c_master_write_test;
        integer wait_count;
        begin
            $display("TEST i2c_master_write");
            i2c_master_rst   = 1'b1;
            i2c_master_start = 1'b0;
            i2c_master_data  = 8'h55;
            step_cycles(3);
            i2c_master_rst = 1'b0;

            i2c_master_start = 1'b1;
            step_cycles(1);
            i2c_master_start = 1'b0;

            wait_count = 0;
            while ((i2c_master_done !== 1'b1) && (wait_count < 300)) begin
                step_cycles(1);
                wait_count = wait_count + 1;
            end

            check_bit(i2c_master_done, 1'b1, "i2c_master_write done");
            check_bit(i2c_master_ack_error, 1'b1, "i2c_master_write detects missing ACK");
        end
    endtask

    // ---------------------------------------------------------------------
    // pcf8574_lcd_writer
    // ---------------------------------------------------------------------
    reg pcf_rst;
    reg pcf_start;
    reg pcf_rs;
    reg pcf_nibble_only;
    reg [7:0] pcf_lcd_data;
    wire pcf_busy;
    wire pcf_done;
    wire pcf_ack_error;
    tri pcf_i2c_scl;
    tri pcf_i2c_sda;
    pullup(pcf_i2c_scl);
    pullup(pcf_i2c_sda);

    pcf8574_lcd_writer #(
        .CLK_HZ   (1_000_000),
        .I2C_HZ   (100_000),
        .I2C_ADDR (7'h27)
    ) u_pcf8574_lcd_writer_tb (
        .clk         (clk),
        .rst         (pcf_rst),
        .start       (pcf_start),
        .rs          (pcf_rs),
        .nibble_only (pcf_nibble_only),
        .lcd_data    (pcf_lcd_data),
        .busy        (pcf_busy),
        .done        (pcf_done),
        .ack_error   (pcf_ack_error),
        .i2c_scl     (pcf_i2c_scl),
        .i2c_sda     (pcf_i2c_sda)
    );

    task run_pcf8574_lcd_writer_test;
        integer wait_count;
        begin
            $display("TEST pcf8574_lcd_writer");
            pcf_rst         = 1'b1;
            pcf_start       = 1'b0;
            pcf_rs          = 1'b1;
            pcf_nibble_only = 1'b0;
            pcf_lcd_data    = 8'h41;
            step_cycles(3);
            pcf_rst = 1'b0;

            pcf_start = 1'b1;
            step_cycles(1);
            pcf_start = 1'b0;

            wait_count = 0;
            while ((pcf_done !== 1'b1) && (wait_count < 5000)) begin
                step_cycles(1);
                wait_count = wait_count + 1;
            end

            check_bit(pcf_done, 1'b1, "pcf8574_lcd_writer done");
            check_bit(pcf_ack_error, 1'b1, "pcf8574_lcd_writer propagates missing ACK");
        end
    endtask

    // ---------------------------------------------------------------------
    // i2c_lcd_1602
    // ---------------------------------------------------------------------
    reg lcd_rst;
    reg [7:0] lcd_temperature;
    reg [7:0] lcd_humidity;
    reg lcd_data_valid;
    wire lcd_ack_error;
    tri lcd_i2c_scl;
    tri lcd_i2c_sda;
    pullup(lcd_i2c_scl);
    pullup(lcd_i2c_sda);

    i2c_lcd_1602 #(
        .CLK_HZ   (100_000),
        .I2C_HZ   (10_000),
        .I2C_ADDR (7'h27)
    ) u_i2c_lcd_1602_tb (
        .clk         (clk),
        .rst         (lcd_rst),
        .temperature (lcd_temperature),
        .humidity    (lcd_humidity),
        .data_valid  (lcd_data_valid),
        .ack_error   (lcd_ack_error),
        .i2c_scl     (lcd_i2c_scl),
        .i2c_sda     (lcd_i2c_sda)
    );

    task run_i2c_lcd_1602_test;
        integer wait_count;
        begin
            $display("TEST i2c_lcd_1602");
            lcd_rst         = 1'b1;
            lcd_temperature = 8'd25;
            lcd_humidity    = 8'd60;
            lcd_data_valid  = 1'b0;
            step_cycles(3);
            lcd_rst = 1'b0;

            wait_count = 0;
            while ((u_i2c_lcd_1602_tb.state !== 6'd23) && (wait_count < 30000)) begin
                step_cycles(1);
                wait_count = wait_count + 1;
            end

            if (u_i2c_lcd_1602_tb.state !== 6'd23)
                fail_msg("i2c_lcd_1602 did not reach ready state");

            lcd_data_valid = 1'b1;
            step_cycles(1);
            lcd_data_valid = 1'b0;
            step_cycles(5);
            check_u8(u_i2c_lcd_1602_tb.temp_latched, 8'd25, "i2c_lcd_1602 temperature latched");
            check_u8(u_i2c_lcd_1602_tb.hum_latched, 8'd60, "i2c_lcd_1602 humidity latched");
        end
    endtask

    // ---------------------------------------------------------------------
    // dht11_reader
    // ---------------------------------------------------------------------
    reg dht_rst;
    tri dht_line;
    reg dht_sensor_low;
    wire [7:0] dht_humidity;
    wire [7:0] dht_temperature;
    wire dht_data_valid;
    wire dht_error;
    pullup(dht_line);
    assign dht_line = dht_sensor_low ? 1'b0 : 1'bz;

    dht11_reader #(
        .CLK_HZ               (1_000_000),
        .POWERUP_DELAY_US     (5),
        .READ_INTERVAL_US     (20),
        .START_LOW_US         (4),
        .RESP_TIMEOUT_US      (120),
        .BIT_LOW_TIMEOUT_US   (120),
        .BIT_HIGH_TIMEOUT_US  (120),
        .BIT_ONE_THRESHOLD_US (12)
    ) u_dht11_reader_tb (
        .clk         (clk),
        .rst         (dht_rst),
        .dht         (dht_line),
        .humidity    (dht_humidity),
        .temperature (dht_temperature),
        .data_valid  (dht_data_valid),
        .error       (dht_error)
    );

    task dht_send_bit;
        input bit_value;
        begin
            dht_sensor_low = 1'b1;
            step_cycles(12);
            dht_sensor_low = 1'b0;
            if (bit_value)
                step_cycles(24);
            else
                step_cycles(6);
        end
    endtask

    task dht_send_byte;
        input [7:0] value;
        integer bit_index;
        begin
            for (bit_index = 7; bit_index >= 0; bit_index = bit_index - 1)
                dht_send_bit(value[bit_index]);
        end
    endtask

    task run_dht11_reader_test;
        integer wait_count;
        reg [7:0] checksum;
        begin
            $display("TEST dht11_reader");
            dht_rst        = 1'b1;
            dht_sensor_low = 1'b0;
            checksum       = 8'd55 + 8'd0 + 8'd23 + 8'd0;
            step_cycles(5);
            dht_rst = 1'b0;

            wait(u_dht11_reader_tb.state == 4'd3);

            dht_sensor_low = 1'b1;
            step_cycles(20);
            dht_sensor_low = 1'b0;
            step_cycles(20);

            dht_send_byte(8'd55);
            dht_send_byte(8'd0);
            dht_send_byte(8'd23);
            dht_send_byte(8'd0);
            dht_send_byte(checksum);

            dht_sensor_low = 1'b1;
            step_cycles(15);
            dht_sensor_low = 1'b0;

            wait_count = 0;
            while ((dht_data_valid !== 1'b1) && (dht_error !== 1'b1) && (wait_count < 500)) begin
                step_cycles(1);
                wait_count = wait_count + 1;
            end

            check_bit(dht_data_valid, 1'b1, "dht11_reader data_valid");
            check_bit(dht_error, 1'b0, "dht11_reader error clear");
            check_u8(dht_humidity, 8'd55, "dht11_reader humidity");
            check_u8(dht_temperature, 8'd23, "dht11_reader temperature");
        end
    endtask

    // ---------------------------------------------------------------------
    // flame_uart_warning
    // ---------------------------------------------------------------------
    reg flame_rst;
    reg flame_in;
    wire flame_sound;
    wire flame_RsTx;

    flame_uart_warning #(
        .CLK_FREQ           (1_000_000),
        .BAUD_RATE          (100_000),
        .FIRE_REPEAT_CYCLES (40),
        .BUZZ_HALF_PERIOD   (3)
    ) u_flame_uart_warning_tb (
        .clk   (clk),
        .rst   (flame_rst),
        .flame (flame_in),
        .sound (flame_sound),
        .RsTx  (flame_RsTx)
    );

    task run_flame_uart_warning_test;
        begin
            $display("TEST flame_uart_warning");
            flame_rst = 1'b1;
            flame_in  = 1'b1;
            step_cycles(3);
            flame_rst = 1'b0;

            force u_flame_uart_warning_tb.fire_detected = 1'b1;
            step_cycles(8);
            if (flame_sound !== 1'b1 && flame_sound !== 1'b0)
                fail_msg("flame_uart_warning sound should be driven");
            if (flame_RsTx !== 1'b0 && flame_RsTx !== 1'b1)
                fail_msg("flame_uart_warning UART should be driven");
            release u_flame_uart_warning_tb.fire_detected;
        end
    endtask

    // ---------------------------------------------------------------------
    // water_level_nonlinear legacy module
    // ---------------------------------------------------------------------
    reg water_legacy_clk_unused;
    reg [11:0] water_legacy_adc;
    wire [3:0] water_legacy_an;
    wire [6:0] water_legacy_seg;

    water_level_nonlinear u_water_level_nonlinear_tb (
        .clk       (clk),
        .water_adc (water_legacy_adc),
        .an        (water_legacy_an),
        .seg       (water_legacy_seg)
    );

    task legacy_apply_stable_adc;
        input [11:0] adc_value;
        begin
            force u_water_level_nonlinear_tb.filter_reg = {2'b00, adc_value, 10'd0};
            step_cycles(1);
        end
    endtask

    task run_water_level_nonlinear_test;
        begin
            $display("TEST water_level_nonlinear");
            water_legacy_adc = 12'd0;
            legacy_apply_stable_adc(12'd50);
            if (u_water_level_nonlinear_tb.water_level !== 4'd0)
                fail_msg("water_level_nonlinear level 0");
            legacy_apply_stable_adc(12'd1700);
            if (u_water_level_nonlinear_tb.water_level !== 4'd2)
                fail_msg("water_level_nonlinear level 2");
            legacy_apply_stable_adc(12'd1800);
            if (u_water_level_nonlinear_tb.water_level !== 4'd4)
                fail_msg("water_level_nonlinear level 4");
            release u_water_level_nonlinear_tb.filter_reg;
        end
    endtask

    // ---------------------------------------------------------------------
    // XXadc_dual_u
    // ---------------------------------------------------------------------
    reg xadc_vauxp7;
    reg xadc_vauxn7;
    wire [11:0] xadc_adc_ch7;

    XXadc_dual_u u_XXadc_dual_u_tb (
        .dclk_in (clk),
        .vauxp7  (xadc_vauxp7),
        .vauxn7  (xadc_vauxn7),
        .adc_ch7 (xadc_adc_ch7)
    );

    task run_XXadc_dual_u_test;
        begin
            $display("TEST XXadc_dual_u");
            xadc_vauxp7 = 1'b0;
            xadc_vauxn7 = 1'b0;
            force u_XXadc_dual_u_tb.xadc_do   = 16'hABC0;
            force u_XXadc_dual_u_tb.xadc_drdy = 1'b1;
            step_cycles(1);
            if (xadc_adc_ch7 !== 12'hABC)
                fail_msg("XXadc_dual_u should latch do_out[15:4]");
            release u_XXadc_dual_u_tb.xadc_do;
            release u_XXadc_dual_u_tb.xadc_drdy;
        end
    endtask

    // ---------------------------------------------------------------------
    // smart_farm_lut
    // ---------------------------------------------------------------------
    reg smart_reset;
    reg smart_echo_in;
    reg smart_vauxp7;
    reg smart_vauxn7;
    wire smart_trig_out;
    wire [3:0] smart_an;
    wire [6:0] smart_seg;
    wire [11:0] smart_led0;
    wire smart_pump_on;

    smart_farm_lut #(
        .CLK_FREQ(1_000_000)
    ) u_smart_farm_lut_tb (
        .clk      (clk),
        .reset    (smart_reset),
        .echo_in  (smart_echo_in),
        .trig_out (smart_trig_out),
        .vauxp7   (smart_vauxp7),
        .vauxn7   (smart_vauxn7),
        .an       (smart_an),
        .seg      (smart_seg),
        .led0     (smart_led0),
        .pump_on  (smart_pump_on)
    );

    task smart_apply_stable_adc;
        input [11:0] adc_value;
        begin
            force u_smart_farm_lut_tb.water_filter_reg = {2'b00, adc_value, 10'd0};
            step_cycles(1);
        end
    endtask

    task run_smart_farm_lut_test;
        begin
            $display("TEST smart_farm_lut");
            smart_reset   = 1'b1;
            smart_echo_in = 1'b0;
            smart_vauxp7  = 1'b0;
            smart_vauxn7  = 1'b0;
            step_cycles(3);
            smart_reset = 1'b0;

            smart_apply_stable_adc(12'd1699);
            check_bit(smart_pump_on, 1'b1, "smart_farm_lut pump on at level 2");
            smart_apply_stable_adc(12'd1700);
            check_bit(smart_pump_on, 1'b1, "smart_farm_lut pump holds on at level 3");
            smart_apply_stable_adc(12'd1850);
            check_bit(smart_pump_on, 1'b0, "smart_farm_lut pump off at level 4");
            smart_apply_stable_adc(12'd1700);
            check_bit(smart_pump_on, 1'b0, "smart_farm_lut pump holds off at level 3");
            smart_apply_stable_adc(12'd1500);
            check_bit(smart_pump_on, 1'b1, "smart_farm_lut pump on again at level 2");

            force u_smart_farm_lut_tb.distance_cm = 10'd3;
            force u_smart_farm_lut_tb.distance_valid = 1'b0;
            step_cycles(1);
            force u_smart_farm_lut_tb.distance_valid = 1'b1;
            step_cycles(1);
            force u_smart_farm_lut_tb.distance_valid = 1'b0;
            check_u10_range(u_smart_farm_lut_tb.plant_length_x10, 10'd0, 10'd0,
                            "smart_farm_lut plant length waits one cycle");
            step_cycles(1);
            check_u10_range(u_smart_farm_lut_tb.plant_length_x10, 10'd50, 10'd50,
                            "smart_farm_lut plant length delayed update");
            if (smart_led0 !== 12'hfff)
                fail_msg("smart_farm_lut LED reflects delayed plant length");

            release u_smart_farm_lut_tb.distance_cm;
            release u_smart_farm_lut_tb.distance_valid;
            release u_smart_farm_lut_tb.water_filter_reg;
        end
    endtask

    // ---------------------------------------------------------------------
    // smartfarm_dht11_basys3 top-level smoke test
    // ---------------------------------------------------------------------
    reg top_btnC;
    tri top_dht11;
    reg top_dht_sensor_low;
    reg top_flame;
    tri top_i2c_scl;
    tri top_i2c_sda;
    reg top_echo_in;
    reg top_vauxp7;
    reg top_vauxn7;
    wire top_servo_out;
    wire top_l298n_in1;
    wire top_sound;
    wire top_RsTx;
    wire top_fan_ena_pwm;
    wire top_fan_in1;
    wire top_fan_in2;
    wire top_trig_out;
    wire [3:0] top_an;
    wire [6:0] top_seg;
    wire [11:0] top_plant_led;
    wire top_pump_on;
    wire top_led_dht_error;
    wire top_led_i2c_error;

    pullup(top_dht11);
    pullup(top_i2c_scl);
    pullup(top_i2c_sda);
    assign top_dht11 = top_dht_sensor_low ? 1'b0 : 1'bz;

    smartfarm_dht11_basys3 #(
        .WINDOW_OPEN_TEMP_C    (8'd31),
        .WINDOW_CLOSE_TEMP_C   (8'd30),
        .HEATER_SET_TEMP_C     (8'd30),
        .HEATER_CUTOFF_TEMP_C  (8'd29),
        .HEATER_RESTART_TEMP_C (8'd28),
        .LCD_I2C_ADDR          (7'h27)
    ) u_smartfarm_dht11_basys3_tb (
        .clk           (clk),
        .btnC          (top_btnC),
        .dht11         (top_dht11),
        .flame         (top_flame),
        .i2c_scl       (top_i2c_scl),
        .i2c_sda       (top_i2c_sda),
        .servo_out     (top_servo_out),
        .l298n_in1     (top_l298n_in1),
        .sound         (top_sound),
        .RsTx          (top_RsTx),
        .fan_ena_pwm   (top_fan_ena_pwm),
        .fan_in1       (top_fan_in1),
        .fan_in2       (top_fan_in2),
        .echo_in       (top_echo_in),
        .trig_out      (top_trig_out),
        .vauxp7        (top_vauxp7),
        .vauxn7        (top_vauxn7),
        .an            (top_an),
        .seg           (top_seg),
        .plant_led     (top_plant_led),
        .pump_on       (top_pump_on),
        .led_dht_error (top_led_dht_error),
        .led_i2c_error (top_led_i2c_error)
    );

    task top_apply_stable_adc;
        input [11:0] adc_value;
        begin
            force u_smartfarm_dht11_basys3_tb.u_smart_farm_lut.water_filter_reg =
                {2'b00, adc_value, 10'd0};
            step_cycles(1);
        end
    endtask

    task run_smartfarm_dht11_basys3_test;
        begin
            $display("TEST smartfarm_dht11_basys3");
            top_btnC           = 1'b1;
            top_dht_sensor_low = 1'b0;
            top_flame          = 1'b1;
            top_echo_in        = 1'b0;
            top_vauxp7         = 1'b0;
            top_vauxn7         = 1'b0;
            step_cycles(4);
            top_btnC = 1'b0;

            top_apply_stable_adc(12'd1699);
            check_bit(top_pump_on, 1'b1, "top pump on path");
            top_apply_stable_adc(12'd1850);
            check_bit(top_pump_on, 1'b0, "top pump off path");
            release u_smartfarm_dht11_basys3_tb.u_smart_farm_lut.water_filter_reg;

            check_bit(top_RsTx, 1'b1, "top UART idle when no fire");
            check_bit(top_sound, 1'b0, "top buzzer off when no fire");
        end
    endtask

    initial begin
        errors = 0;

        tick_rst = 1'b1;
        uart_rst = 1'b1;
        uart_start = 1'b0;
        ultra_reset = 1'b1;
        ultra_start = 1'b0;
        ultra_echo = 1'b0;
        fan_rst = 1'b1;
        fan_valid = 1'b0;
        fan_humidity = 8'd0;
        heater_rst = 1'b1;
        heater_valid = 1'b0;
        heater_error = 1'b0;
        heater_temperature = 8'd30;
        servo_rst = 1'b1;
        servo_valid = 1'b0;
        servo_temperature = 8'd0;
        servo_force_closed = 1'b0;
        i2c_master_rst = 1'b1;
        i2c_master_start = 1'b0;
        i2c_master_data = 8'd0;
        pcf_rst = 1'b1;
        pcf_start = 1'b0;
        pcf_rs = 1'b0;
        pcf_nibble_only = 1'b0;
        pcf_lcd_data = 8'd0;
        lcd_rst = 1'b1;
        lcd_temperature = 8'd0;
        lcd_humidity = 8'd0;
        lcd_data_valid = 1'b0;
        dht_rst = 1'b1;
        dht_sensor_low = 1'b0;
        flame_rst = 1'b1;
        flame_in = 1'b1;
        water_legacy_adc = 12'd0;
        xadc_vauxp7 = 1'b0;
        xadc_vauxn7 = 1'b0;
        smart_reset = 1'b1;
        smart_echo_in = 1'b0;
        smart_vauxp7 = 1'b0;
        smart_vauxn7 = 1'b0;
        top_btnC = 1'b1;
        top_dht_sensor_low = 1'b0;
        top_flame = 1'b1;
        top_echo_in = 1'b0;
        top_vauxp7 = 1'b0;
        top_vauxn7 = 1'b0;

        step_cycles(5);

        run_tick_1us_gen_test();
        run_uart_tx_byte_test();
        run_ultrasonic_distance_meter_once_test();
        run_l298_humidity_fan_controller_test();
        run_l298n_heater_controller_test();
        run_servo_window_control_test();
        run_i2c_master_write_test();
        run_pcf8574_lcd_writer_test();
        run_i2c_lcd_1602_test();
        run_dht11_reader_test();
        run_flame_uart_warning_test();
        run_water_level_nonlinear_test();
        run_XXadc_dual_u_test();
        run_smart_farm_lut_test();
        run_smartfarm_dht11_basys3_test();

        if (errors == 0)
            $display("PASS all module testbenches");
        else
            $display("FAIL all module testbenches errors=%0d", errors);

        $finish;
    end

endmodule
