`timescale 1ns / 1ps


module smartfarm_dht11_basys3 #(
    parameter [7:0] WINDOW_OPEN_TEMP_C  = 8'd31,
    parameter [7:0] WINDOW_CLOSE_TEMP_C = 8'd30,
    parameter [7:0] HEATER_SET_TEMP_C   = 8'd30,
    parameter [7:0] HEATER_CUTOFF_TEMP_C = 8'd29,
    parameter [7:0] HEATER_RESTART_TEMP_C = 8'd28,
    parameter [6:0] LCD_I2C_ADDR        = 7'h27
)(
    input  wire clk,        // Basys3 100 MHz
    input  wire btnC,       // reset, active high

    inout  wire dht11,      // DHT11 DATA
    input  wire flame,      // active-low flame sensor output

    // 1602 LCD I2C interface (open-drain)
    inout  wire i2c_scl,
    inout  wire i2c_sda,

    output wire servo_out,
    output wire l298n_in1,
    output wire sound,
    output wire RsTx,

    output wire fan_ena_pwm,
    output wire fan_in1,
    output wire fan_in2,

    // smart_farm_lut sensors/display
    input  wire echo_in,
    output wire trig_out,
    input  wire vauxp7,
    input  wire vauxn7,
    output wire [3:0] an,
    output wire [6:0] seg,
    output wire [11:0] plant_led,
    output wire pump_on,

    // 디버그 LED
    output wire led_dht_error,
    output wire led_i2c_error
);

    // -------------------------------------------------------------------------
    // DHT11 data
    // -------------------------------------------------------------------------
    wire [7:0] humidity;
    wire [7:0] temperature;
    wire       data_valid;
    wire       dht_error;

    dht11_reader #(
        .CLK_HZ            (100000000),
        .POWERUP_DELAY_US  (1000000),
        .READ_INTERVAL_US  (2000000)
    ) u_dht11_reader (
        .clk          (clk),
        .rst          (btnC),
        .dht          (dht11),
        .humidity     (humidity),
        .temperature  (temperature),
        .data_valid   (data_valid),
        .error        (dht_error)
    );

    // -------------------------------------------------------------------------
    // LCD update pulse adapter
    //
    // data_valid가 계속 HIGH인 구조를 그대로 LCD에 연결하면 data_toggle이
    // 매 클럭 반전될 수 있다. 첫 정상 데이터 또는 값이 바뀐 경우에만
    // 한 클럭 폭의 lcd_update_pulse를 만든다.
    // -------------------------------------------------------------------------
    reg [7:0] last_lcd_temperature;
    reg [7:0] last_lcd_humidity;
    reg       lcd_has_data;
    reg       lcd_update_pulse;

    always @(posedge clk) begin
        if (btnC) begin
            last_lcd_temperature <= 8'd0;
            last_lcd_humidity    <= 8'd0;
            lcd_has_data         <= 1'b0;
            lcd_update_pulse     <= 1'b0;
        end else begin
            lcd_update_pulse <= 1'b0;

            if (data_valid &&
                (!lcd_has_data ||
                 (temperature != last_lcd_temperature) ||
                 (humidity    != last_lcd_humidity))) begin
                last_lcd_temperature <= temperature;
                last_lcd_humidity    <= humidity;
                lcd_has_data         <= 1'b1;
                lcd_update_pulse     <= 1'b1;
            end
        end
    end

    wire i2c_ack_error;

    i2c_lcd_1602 #(
        .CLK_HZ   (100000000),
        .I2C_HZ   (100000),
        .I2C_ADDR (LCD_I2C_ADDR)
    ) u_i2c_lcd_1602 (
        .clk          (clk),
        .rst          (btnC),
        .temperature  (temperature),
        .humidity     (humidity),
        .data_valid   (lcd_update_pulse),
        .ack_error    (i2c_ack_error),
        .i2c_scl      (i2c_scl),
        .i2c_sda      (i2c_sda)
    );

    // I2C ACK 오류는 짧게 발생할 수 있으므로 LED 확인용으로 래치한다.
    // btnC를 누르면 해제된다.
    reg i2c_error_latched;

    always @(posedge clk) begin
        if (btnC)
            i2c_error_latched <= 1'b0;
        else if (i2c_ack_error)
            i2c_error_latched <= 1'b1;
    end

    assign led_dht_error = dht_error;
    assign led_i2c_error = i2c_error_latched;

    // -------------------------------------------------------------------------
    // Flame input synchronizer and forced servo close
    // -------------------------------------------------------------------------
    reg flame_meta;
    reg flame_sync;

    always @(posedge clk) begin
        if (btnC) begin
            flame_meta <= 1'b1;
            flame_sync <= 1'b1;
        end else begin
            flame_meta <= flame;
            flame_sync <= flame_meta;
        end
    end

    localparam integer FLAME_FILTER_CYCLES = 3000000;

    reg [31:0] flame_low_count;
    reg        fire_detected;

    always @(posedge clk) begin
        if (btnC) begin
            flame_low_count <= 32'd0;
            fire_detected   <= 1'b0;
        end else if (flame_sync == 1'b0) begin
            if (flame_low_count < FLAME_FILTER_CYCLES - 1) begin
                flame_low_count <= flame_low_count + 1'b1;
            end else begin
                fire_detected <= 1'b1;
            end
        end else begin
            flame_low_count <= 32'd0;
            fire_detected   <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // Servo window control
    // -------------------------------------------------------------------------
    servo_window_control #(
        .CLK_HZ           (100000000),
        .OPEN_TEMP_C      (WINDOW_OPEN_TEMP_C),
        .CLOSE_TEMP_C     (WINDOW_CLOSE_TEMP_C),
        .CLOSED_PULSE_US  (1000),
        .OPEN_PULSE_US    (2000)
    ) u_servo_window_control (
        .clk           (clk),
        .rst           (btnC),
        .valid         (data_valid && !dht_error),
        .temperature   (temperature),
        .force_closed  (fire_detected),
        .servo_pwm     (servo_out)
    );

    // -------------------------------------------------------------------------
    // Flame warning: buzzer + UART
    // -------------------------------------------------------------------------
    wire flame_RsTx;

    assign RsTx = flame_RsTx;

    flame_uart_warning #(
        .CLK_FREQ            (100000000),
        .BAUD_RATE           (9600),
        .FIRE_REPEAT_CYCLES  (100000000),
        .BUZZ_HALF_PERIOD    (20833)
    ) u_flame_uart_warning (
        .clk    (clk),
        .rst    (btnC),
        .flame  (flame),
        .sound  (sound),
        .RsTx   (flame_RsTx)
    );

    // -------------------------------------------------------------------------
    // Humidity fan control
    // -------------------------------------------------------------------------
    l298_humidity_fan_controller #(
        .CLK_FREQ_HZ        (100000000),
        .PWM_FREQ_HZ        (1000),
        .HUM_START_PERCENT  (8'd80),
        .HUM_FULL_PERCENT   (8'd100),
        .MIN_DUTY           (8'd128)
    ) u_l298_humidity_fan_controller (
        .clk             (clk),
        .rst             (btnC),
        .valid           (data_valid && !dht_error),
        .humidity        (humidity),
        .fan_ena_pwm     (fan_ena_pwm),
        .fan_in1         (fan_in1),
        .fan_in2         (fan_in2),
        .fan_duty_debug  ()
    );

    // -------------------------------------------------------------------------
    // L298N heater PI PWM control
    // -------------------------------------------------------------------------
    l298n_heater_controller #(
        .CLK_HZ             (100000000),
        .SET_TEMP_C         (HEATER_SET_TEMP_C),
        .CUTOFF_TEMP_C      (HEATER_CUTOFF_TEMP_C),
        .RESTART_TEMP_C     (HEATER_RESTART_TEMP_C),
        .KP                 (16'sd80),
        .KI                 (16'sd10),
        .INTEGRAL_MIN       (32'sd0),
        .INTEGRAL_MAX       (32'sd100),
        .MAX_DUTY_PERMILLE  (250),
        .PWM_PERIOD_MS      (1000),
        .PI_SAMPLE_MS       (2000)
    ) u_l298n_heater_controller (
        .clk          (clk),
        .rst          (btnC),
        .valid        (data_valid && !dht_error),
        .error        (dht_error),
        .temperature  (temperature),
        .l298n_in1    (l298n_in1)
    );

    // -------------------------------------------------------------------------
    // Water level + ultrasonic plant length LUT system
    // -------------------------------------------------------------------------
    smart_farm_lut #(
        .CLK_FREQ  (100000000)
    ) u_smart_farm_lut (
        .clk        (clk),
        .reset      (btnC),
        .echo_in    (echo_in),
        .trig_out   (trig_out),
        .vauxp7     (vauxp7),
        .vauxn7     (vauxn7),
        .an         (an),
        .seg        (seg),
        .led0       (plant_led),
        .pump_on    (pump_on)
    );

endmodule
