`timescale 1ns / 1ps

module XXadc_dual_u (
    input  wire       dclk_in,
    input  wire       vauxp7,
    input  wire       vauxn7,
    output reg [11:0] adc_ch7
);

    wire [15:0] xadc_do;
    wire        xadc_drdy;
    wire        xadc_eoc;

    xadc_wiz_0 u_xadc_wiz_0 (
        .di_in              (16'h0000),
        .daddr_in           (7'h17),
        .den_in             (xadc_eoc),
        .dwe_in             (1'b0),
        .drdy_out           (xadc_drdy),
        .do_out             (xadc_do),
        .dclk_in            (dclk_in),
        .reset_in           (1'b0),
        .vp_in              (1'b0),
        .vn_in              (1'b0),
        .vauxp7             (vauxp7),
        .vauxn7             (vauxn7),
        .user_temp_alarm_out(),
        .vccint_alarm_out   (),
        .vccaux_alarm_out   (),
        .ot_out             (),
        .channel_out        (),
        .eoc_out            (xadc_eoc),
        .alarm_out          (),
        .eos_out            (),
        .busy_out           ()
    );

    always @(posedge dclk_in) begin
        if (xadc_drdy)
            adc_ch7 <= xadc_do[15:4];
    end

endmodule
