# ============================================================
# OV7670 + FIFO Camera
# Zybo Z7-20 Rev.B
#
# Notes:
# - Keep AXI register bit mapping unchanged.
# - VSYNC/HREF weak pulldowns are intentionally not used while debugging.
#   The camera should actively drive these signals, and pulldowns can mask
#   weak or marginal outputs.
# ============================================================


# ------------------------------------------------------------
# JD : Camera Data D0 ~ D7
# ------------------------------------------------------------

set_property PACKAGE_PIN T14 [get_ports {camera_data_0[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_data_0[0]}]

set_property PACKAGE_PIN T15 [get_ports {camera_data_0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_data_0[1]}]

set_property PACKAGE_PIN P14 [get_ports {camera_data_0[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_data_0[2]}]

set_property PACKAGE_PIN R14 [get_ports {camera_data_0[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_data_0[3]}]

set_property PACKAGE_PIN U14 [get_ports {camera_data_0[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_data_0[4]}]

set_property PACKAGE_PIN U15 [get_ports {camera_data_0[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_data_0[5]}]

set_property PACKAGE_PIN V17 [get_ports {camera_data_0[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_data_0[6]}]

set_property PACKAGE_PIN V18 [get_ports {camera_data_0[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_data_0[7]}]


# ------------------------------------------------------------
# JC : Camera Control / Sync
# ------------------------------------------------------------

# JC1 -> VSYNC
set_property PACKAGE_PIN V15 [get_ports {camera_vsync_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_vsync_0}]

# JC2 -> HREF
set_property PACKAGE_PIN W15 [get_ports {camera_href_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_href_0}]

# JC3 -> WR
set_property PACKAGE_PIN T11 [get_ports {camera_wr_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_wr_0}]

# JC4 -> WRST
set_property PACKAGE_PIN T10 [get_ports {camera_wrst_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_wrst_0}]

# JC7 -> RRST
set_property PACKAGE_PIN W14 [get_ports {camera_rrst_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_rrst_0}]

# JC8 -> RCLK
set_property PACKAGE_PIN Y14 [get_ports {camera_rclk_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_rclk_0}]

# JC9 -> SIOC
set_property PACKAGE_PIN T12 [get_ports {camera_sioc_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_sioc_0}]

# JC10 -> SIOD
set_property PACKAGE_PIN U12 [get_ports {camera_siod_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_siod_0}]

# SIOD internal weak pull-up for SCCB idle.
set_property PULLUP true [get_ports {camera_siod_0}]


# ============================================================
# PMOD JE
# ============================================================

# JE1 -> MG996R Servo PWM
set_property PACKAGE_PIN V12 [get_ports {servo_pwm_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {servo_pwm_0}]

# JE2 -> OV7670 FIFO OE
set_property PACKAGE_PIN W16 [get_ports {camera_oe_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_oe_0}]

# JE3 -> OV7670 RST
set_property PACKAGE_PIN J15 [get_ports {camera_rst_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {camera_rst_0}]
