## I2C signals for MIPI 0
#set_property PACKAGE_PIN AW21 [get_ports iic_0_scl_io]; # LA03_N
#set_property PACKAGE_PIN AV22 [get_ports iic_0_sda_io]; # LA03_P
#set_property IOSTANDARD LVCMOS12 [get_ports iic_0_*]
#set_property SLEW SLOW [get_ports iic_0_*]
#set_property DRIVE 4 [get_ports iic_0_*]

# I2C signals for MIPI 1
set_property PACKAGE_PIN BG23 [get_ports iic_1_scl_io]; # LA05_N
set_property PACKAGE_PIN BF24 [get_ports iic_1_sda_io]; # LA05_P
set_property IOSTANDARD LVCMOS12 [get_ports iic_1_*]
set_property SLEW SLOW [get_ports iic_1_*]
set_property DRIVE 4 [get_ports iic_1_*]

# I2C signals for MIPI 2
set_property PACKAGE_PIN AR20 [get_ports iic_2_scl_io]; # LA30_N
set_property PACKAGE_PIN AT20 [get_ports iic_2_sda_io]; # LA30_P
set_property IOSTANDARD LVCMOS12 [get_ports iic_2_*]
set_property SLEW SLOW [get_ports iic_2_*]
set_property DRIVE 4 [get_ports iic_2_*]

## I2C signals for MIPI 3
#set_property PACKAGE_PIN AN19 [get_ports iic_3_scl_io]; # LA32_N
#set_property PACKAGE_PIN AN20 [get_ports iic_3_sda_io]; # LA32_P
#set_property IOSTANDARD LVCMOS12 [get_ports iic_3_*]
#set_property SLEW SLOW [get_ports iic_3_*]
#set_property DRIVE 4 [get_ports iic_3_*]

# CAM1 and CAM3 CLK_SEL signals
set_property PACKAGE_PIN BG16 [get_ports {clk_sel[0]}]; # LA25_N
set_property IOSTANDARD LVCMOS12 [get_ports {clk_sel[0]}]

set_property PACKAGE_PIN BF16 [get_ports {clk_sel[1]}]; # LA25_P
set_property IOSTANDARD LVCMOS12 [get_ports {clk_sel[1]}]

## GPIOs for MIPI camera 0
#set_property PACKAGE_PIN BF22 [get_ports {gpio_0_tri_o[0]}]; # LA12_N
#set_property PACKAGE_PIN BG21 [get_ports {gpio_0_tri_o[1]}]; # LA12_P
#set_property IOSTANDARD LVCMOS12 [get_ports {gpio_0_tri_o[*]}]

# GPIOs for MIPI camera 1
set_property PACKAGE_PIN BE24 [get_ports {gpio_1_tri_o[0]}]; # LA09_N
set_property PACKAGE_PIN BE25 [get_ports {gpio_1_tri_o[1]}]; # LA09_P
set_property IOSTANDARD LVCMOS12 [get_ports {gpio_1_tri_o[*]}]

# GPIOs for MIPI camera 2
set_property PACKAGE_PIN BA16 [get_ports {gpio_2_tri_o[0]}]; # LA19_N
set_property PACKAGE_PIN BA17 [get_ports {gpio_2_tri_o[1]}]; # LA19_P
set_property IOSTANDARD LVCMOS12 [get_ports {gpio_2_tri_o[*]}]

## GPIOs for MIPI camera 3
#set_property PACKAGE_PIN BF17 [get_ports {gpio_3_tri_o[0]}]; # LA20_N
#set_property PACKAGE_PIN BE16 [get_ports {gpio_3_tri_o[1]}]; # LA20_P
#set_property IOSTANDARD LVCMOS12 [get_ports {gpio_3_tri_o[*]}]

# Reserved GPIOs
set_property PACKAGE_PIN AU21 [get_ports {rsvd_gpio_tri_o[0]}]; # LA04_P
set_property PACKAGE_PIN AV21 [get_ports {rsvd_gpio_tri_o[1]}]; # LA04_N
set_property PACKAGE_PIN BC25 [get_ports {rsvd_gpio_tri_o[2]}]; # LA07_P
set_property PACKAGE_PIN BD25 [get_ports {rsvd_gpio_tri_o[3]}]; # LA07_N
set_property PACKAGE_PIN BE21 [get_ports {rsvd_gpio_tri_o[4]}]; # LA13_P
set_property PACKAGE_PIN BE20 [get_ports {rsvd_gpio_tri_o[5]}]; # LA13_N
set_property PACKAGE_PIN BF19 [get_ports {rsvd_gpio_tri_o[6]}]; # LA27_P
set_property PACKAGE_PIN BG19 [get_ports {rsvd_gpio_tri_o[7]}]; # LA27_N
set_property PACKAGE_PIN AM21 [get_ports {rsvd_gpio_tri_o[8]}]; # LA29_P
set_property PACKAGE_PIN AM20 [get_ports {rsvd_gpio_tri_o[9]}]; # LA29_N
set_property IOSTANDARD LVCMOS12 [get_ports {rsvd_gpio_tri_o[*]}]

## MIPI interface 0
#set_property PACKAGE_PIN BD23 [get_ports {mipi_phy_if_0_clk_p}]; # LA00_CC_P
#set_property PACKAGE_PIN BD24 [get_ports {mipi_phy_if_0_clk_n}]; # LA00_CC_N
#set_property PACKAGE_PIN BC20 [get_ports {mipi_phy_if_0_data_p[0]}]; # LA06_P
#set_property PACKAGE_PIN BD20 [get_ports {mipi_phy_if_0_data_n[0]}]; # LA06_N
#set_property PACKAGE_PIN AW24 [get_ports {mipi_phy_if_0_data_p[1]}]; # LA02_P
#set_property PACKAGE_PIN AY25 [get_ports {mipi_phy_if_0_data_n[1]}]; # LA02_N

#set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_0_clk_p]
#set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_0_clk_n]
#set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_0_data_p[*]]
#set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_0_data_n[*]]

#set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_0_clk_p]
#set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_0_clk_n]
#set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_0_data_p[*]]
#set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_0_data_n[*]]

# MIPI interface 1
set_property PACKAGE_PIN BC23 [get_ports {mipi_phy_if_1_clk_p}]; # LA01_CC_P
set_property PACKAGE_PIN BD22 [get_ports {mipi_phy_if_1_clk_n}]; # LA01_CC_N
set_property PACKAGE_PIN AY22 [get_ports {mipi_phy_if_1_data_p[0]}]; # LA15_P
set_property PACKAGE_PIN AY23 [get_ports {mipi_phy_if_1_data_n[0]}]; # LA15_N
set_property PACKAGE_PIN AU24 [get_ports {mipi_phy_if_1_data_p[1]}]; # LA14_P
set_property PACKAGE_PIN AU23 [get_ports {mipi_phy_if_1_data_n[1]}]; # LA14_N

set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_1_clk_p]
set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_1_clk_n]
set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_1_data_p[*]]
set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_1_data_n[*]]

set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_1_clk_p]
set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_1_clk_n]
set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_1_data_p[*]]
set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_1_data_n[*]]

# MIPI interface 2
set_property PACKAGE_PIN BE17 [get_ports {mipi_phy_if_2_clk_p}]; # LA18_CC_P
set_property PACKAGE_PIN BD17 [get_ports {mipi_phy_if_2_clk_n}]; # LA18_CC_N
set_property PACKAGE_PIN BA20 [get_ports {mipi_phy_if_2_data_p[0]}]; # LA24_P
set_property PACKAGE_PIN BA19 [get_ports {mipi_phy_if_2_data_n[0]}]; # LA24_N
set_property PACKAGE_PIN BB16 [get_ports {mipi_phy_if_2_data_p[1]}]; # LA17_CC_P
set_property PACKAGE_PIN BC16 [get_ports {mipi_phy_if_2_data_n[1]}]; # LA17_CC_N

set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_2_clk_p]
set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_2_clk_n]
set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_2_data_p[*]]
set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_2_data_n[*]]

set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_2_clk_p]
set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_2_clk_n]
set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_2_data_p[*]]
set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_2_data_n[*]]

## MIPI interface 3
#set_property PACKAGE_PIN AR18 [get_ports {mipi_phy_if_3_clk_p}]; # LA31_P
#set_property PACKAGE_PIN AT19 [get_ports {mipi_phy_if_3_clk_n}]; # LA31_N
#set_property PACKAGE_PIN AU20 [get_ports {mipi_phy_if_3_data_p[0]}]; # LA33_P
#set_property PACKAGE_PIN AU19 [get_ports {mipi_phy_if_3_data_n[0]}]; # LA33_N
#set_property PACKAGE_PIN BB18 [get_ports {mipi_phy_if_3_data_p[1]}]; # LA28_P
#set_property PACKAGE_PIN BC17 [get_ports {mipi_phy_if_3_data_n[1]}]; # LA28_N

#set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_3_clk_p]
#set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_3_clk_n]
#set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_3_data_p[*]]
#set_property IOSTANDARD MIPI_DPHY [get_ports mipi_phy_if_3_data_n[*]]

#set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_3_clk_p]
#set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_3_clk_n]
#set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_3_data_p[*]]
#set_property DIFF_TERM_ADV TERM_100 [get_ports mipi_phy_if_3_data_n[*]]

# HDMI TX
set_property PACKAGE_PIN F41 [get_ports TX_DATA_OUT_txp[0]]
set_property PACKAGE_PIN D41 [get_ports TX_DATA_OUT_txp[1]]
set_property PACKAGE_PIN B41 [get_ports TX_DATA_OUT_txp[2]]
set_property PACKAGE_PIN A43 [get_ports TX_DATA_OUT_txp[3]]

set_property PACKAGE_PIN F46 [get_ports RX_DATA_IN_rxp[0]]
set_property PACKAGE_PIN E44 [get_ports RX_DATA_IN_rxp[1]]
set_property PACKAGE_PIN D46 [get_ports RX_DATA_IN_rxp[2]]
set_property PACKAGE_PIN C44 [get_ports RX_DATA_IN_rxp[3]]

#HDMI_8T49N241_OUT_C_P
set_property PACKAGE_PIN E39 [get_ports TX_REFCLK_P_IN]
#HDMI_TX_HPD
set_property PACKAGE_PIN K18 [get_ports TX_HPD_IN]
set_property IOSTANDARD LVCMOS33 [get_ports TX_HPD_IN]
#HDMI_TX_SRC_SCL
set_property PACKAGE_PIN K21 [get_ports TX_DDC_OUT_scl_io]
set_property IOSTANDARD LVCMOS33 [get_ports TX_DDC_OUT_scl_io]
#HDMI_TX_SRC_SDA
set_property PACKAGE_PIN L20 [get_ports TX_DDC_OUT_sda_io]
set_property IOSTANDARD LVCMOS33 [get_ports TX_DDC_OUT_sda_io]

# I2C
#HDMI_CTL_SCL
set_property IOSTANDARD LVCMOS33 [get_ports HDMI_CTL_IIC_scl_io]
set_property PACKAGE_PIN K17 [get_ports HDMI_CTL_IIC_scl_io]
#HDMI_CTL_SDA
set_property IOSTANDARD LVCMOS33 [get_ports HDMI_CTL_IIC_sda_io]
set_property PACKAGE_PIN L18 [get_ports HDMI_CTL_IIC_sda_io]

#HDMI_8T49N241_LOL
set_property PACKAGE_PIN G20 [get_ports IDT_8T49N241_LOL_IN]
set_property IOSTANDARD LVCMOS33 [get_ports IDT_8T49N241_LOL_IN]
#HDMI_TX_EN
set_property PACKAGE_PIN K20 [get_ports TX_EN_OUT]
set_property IOSTANDARD LVCMOS33 [get_ports TX_EN_OUT]

#HDMI AUDIO
set_property PACKAGE_PIN AE42 [get_ports CLK_IN_AUDIO_clk_p[0]]       
set_property IOSTANDARD DIFF_LVSTL_11 [get_ports CLK_IN_AUDIO_clk_p[0]]

# PL Clock uncertainty
set_clock_uncertainty -hold  0.050 -from [get_clocks *pl_0] -to [get_clocks *pl_0]

################
# Clock Groups #
################
#
create_clock -period 3.367 -name hdmi_tx_clk [get_ports TX_REFCLK_P_IN]
create_clock -period 10.000 -name audio_ref_clk [get_ports CLK_IN_AUDIO_clk_p]

set_false_path -from [get_cells -hier -filter name=~*HDMI_ACR_CTRL_AXI_INST/rEnab_ACR_reg] -to [get_cells -hier -filter {name=~*aud_enab_acr_sync_reg[0]}]
set_false_path -from [get_cells -hier -filter name=~*HDMI_ACR_CTRL_AXI_INST/rACR_Sel_reg] -to [get_cells -hier -filter {name=~*aud_acr_sel_sync_reg[0]}]
set_false_path -from [get_cells -hier -filter name=~*HDMI_ACR_CTRL_AXI_INST/rTMDSClkRatio_reg] -to [get_cells -hier -filter {name=~*aud_tmdsclkratio_sync_reg[0]}]
set_false_path -from [get_cells -hier -filter name=~*PULSE_CLKCROSS_INST/rIn_Toggle_reg] -to [get_cells -hier -filter {name=~*PULSE_CLKCROSS_INST/rOut_Sync_reg[0]}]
set_false_path -from [get_cells -hier -filter name=~*HDMI_ACR_CTRL_AXI_INST/rAud_Reset_reg] -to [get_cells -hier -filter {name=~*aud_rst_chain_reg[*]}]
set_false_path -from [get_cells -hier -filter {name=~*NVAL_CLKCROSS_INST/rIn_Data_reg[*]}] -to [get_cells -hier -filter {name=~*NVAL_CLKCROSS_INST/rOut_Data_reg[*]}]
set_false_path -from [get_cells -hier -filter name=~*NVAL_CLKCROSS_INST/rIn_DValid_reg] -to [get_cells -hier -filter {name=~*NVAL_CLKCROSS_INST/rOut_DValid_Sync_reg[0]}]
set_false_path -from [get_cells -hier -filter name=~*NVAL_CLKCROSS_INST/rOut_ACK_reg] -to [get_cells -hier -filter {name=~*NVAL_CLKCROSS_INST/rIn_ACK_Sync_reg[0]}]
set_false_path -from [get_cells -hier -filter {name=~*CTS_CLKCROSS_ACLK_INST/rIn_Data_reg[*]}] -to [get_cells -hier -filter {name=~*CTS_CLKCROSS_ACLK_INST/rOut_Data_reg[*]}]
set_false_path -from [get_cells -hier -filter name=~*CTS_CLKCROSS_ACLK_INST/rIn_DValid_reg] -to [get_cells -hier -filter {name=~*CTS_CLKCROSS_ACLK_INST/rOut_DValid_Sync_reg[0]}]
set_false_path -from [get_cells -hier -filter name=~*CTS_CLKCROSS_ACLK_INST/rOut_ACK_reg] -to [get_cells -hier -filter {name=~*CTS_CLKCROSS_ACLK_INST/rIn_ACK_Sync_reg[0]}]
set_false_path -from [get_cells -hier -filter {name=~*CTS_CLKCROSS_AUD_INST/rIn_Data_reg[*]}] -to [get_cells -hier -filter {name=~*CTS_CLKCROSS_AUD_INST/rOut_Data_reg[*]}]
set_false_path -from [get_cells -hier -filter name=~*CTS_CLKCROSS_AUD_INST/rIn_DValid_reg] -to [get_cells -hier -filter {name=~*CTS_CLKCROSS_AUD_INST/rOut_DValid_Sync_reg[0]}]
set_false_path -from [get_cells -hier -filter name=~*CTS_CLKCROSS_AUD_INST/rOut_ACK_reg] -to [get_cells -hier -filter {name=~*CTS_CLKCROSS_AUD_INST/rIn_ACK_Sync_reg[0]}]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

