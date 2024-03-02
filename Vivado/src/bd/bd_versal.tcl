################################################################
# Block design build script for Versal designs
################################################################

################################################################
# 32x LPD GPIOs are connected as follows:
################################################################
# 0  - Video Mixer IP (display_pipe)
# 1-7 Reserved
# 8  - CAM0 ISPPipeline IP reset
# 9  - CAM0 Vproc IP reset
# 10 - CAM0 Frame Buffer Write IP reset
# 11 - Reserved
# 12 - CAM1 ISPPipeline IP reset
# 13 - CAM1 Vproc IP reset
# 14 - CAM1 Frame Buffer Write IP reset
# 15 - Reserved
# 16 - CAM2 ISPPipeline IP reset
# 17 - CAM2 Vproc IP reset
# 18 - CAM2 Frame Buffer Write IP reset
# 19 - Reserved
# 20 - CAM3 ISPPipeline IP reset
# 21 - CAM3 Vproc IP reset
# 22 - CAM3 Frame Buffer Write IP reset
# 23-31 Reserved

# CHECKING IF PROJECT EXISTS
if { [get_projects -quiet] eq "" } {
   puts "ERROR: Please open or create a project!"
   return 1
}

set cur_design [current_bd_design -quiet]
set list_cells [get_bd_cells -quiet]

create_bd_design $block_name

current_bd_design $block_name

set parentCell [get_bd_cells /]

# Get object for parentCell
set parentObj [get_bd_cells $parentCell]
if { $parentObj == "" } {
   puts "ERROR: Unable to find parent cell <$parentCell>!"
   return
}

# Make sure parentObj is hier blk
set parentType [get_property TYPE $parentObj]
if { $parentType ne "hier" } {
   puts "ERROR: Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."
   return
}

# Save current instance; Restore later
set oldCurInst [current_bd_instance .]

# Set parent object as current
current_bd_instance $parentObj

# Returns true if str contains substr
proc str_contains {str substr} {
  if {[string first $substr $str] == -1} {
    return 0
  } else {
    return 1
  }
}
set cams { 1 2 }
# List of interrupt pins
set intr_list {}

# AXI Lite ports
set noc_ports {}
set m_axi_fpd_ports {}

# Number of cameras
set num_cams [llength $cams]

# Video pipe max res has impact on resource usage
set max_cols 1920
set max_rows 1232

# Set the samples-per-clock for the video pipelines (1)
# Set this to 2 to double the throughput at the cost of higher resource usage
set samples_pc 1

# Add the CIPS
create_bd_cell -type ip -vlnv xilinx.com:ip:versal_cips versal_cips_0

# Configure the CIPS using automation feature
apply_bd_automation -rule xilinx.com:bd_rule:cips -config { \
  board_preset {Yes} \
  boot_config {Custom} \
  configure_noc {Add new AXI NoC} \
  debug_config {JTAG} \
  design_flow {Full System} \
  mc_type {LPDDR} \
  num_mc_ddr {None} \
  num_mc_lpddr {2} \
  pl_clocks {1} \
  pl_resets {1} \
}  [get_bd_cells versal_cips_0]

# Extra config for this design
set_property -dict [list \
  CONFIG.PS_PMC_CONFIG { \
  CLOCK_MODE Custom  \
  DDR_MEMORY_MODE {Connectivity to DDR via NOC}  \
  DEBUG_MODE JTAG  \
  DESIGN_MODE 1  \
  PMC_CRP_PL0_REF_CTRL_FREQMHZ 100  \
  PMC_GPIO0_MIO_PERIPHERAL {{ENABLE 1} {IO {PMC_MIO 0 .. 25}}}  \
  PMC_GPIO1_MIO_PERIPHERAL {{ENABLE 1} {IO {PMC_MIO 26 .. 51}}}  \
  PMC_MIO37 {{AUX_IO 0} {DIRECTION out} {DRIVE_STRENGTH 8mA} {OUTPUT_DATA high} {PULL pullup} {SCHMITT 0} {SLEW slow} {USAGE GPIO}}  \
  PMC_OSPI_PERIPHERAL {{ENABLE 0} {IO {PMC_MIO 0 .. 11}} {MODE Single}}  \
  PMC_QSPI_COHERENCY 0  PMC_QSPI_FBCLK {{ENABLE 1} {IO {PMC_MIO 6}}}  \
  PMC_QSPI_PERIPHERAL_DATA_MODE x4  \
  PMC_QSPI_PERIPHERAL_ENABLE 1  \
  PMC_QSPI_PERIPHERAL_MODE {Dual Parallel}  \
  PMC_REF_CLK_FREQMHZ 33.3333  \
  PMC_SD1 {{CD_ENABLE 1} {CD_IO {PMC_MIO 28}} {POW_ENABLE 1} {POW_IO {PMC_MIO 51}} {RESET_ENABLE 0} {RESET_IO {PMC_MIO 12}} {WP_ENABLE 0} {WP_IO {PMC_MIO 1}}}  \
  PMC_SD1_COHERENCY 0  \
  PMC_SD1_DATA_TRANSFER_MODE 8Bit  \
  PMC_SD1_PERIPHERAL {{CLK_100_SDR_OTAP_DLY 0x3} {CLK_200_SDR_OTAP_DLY 0x2} {CLK_50_DDR_ITAP_DLY 0x36} {CLK_50_DDR_OTAP_DLY 0x3} {CLK_50_SDR_ITAP_DLY 0x2C} {CLK_50_SDR_OTAP_DLY 0x4} {ENABLE 1} {IO {PMC_MIO 26 .. 36}}}  \
  PMC_SD1_SLOT_TYPE {SD 3.0}  \
  PMC_USE_PMC_NOC_AXI0 1  \
  PS_BOARD_INTERFACE ps_pmc_fixed_io  \
  PS_CAN1_PERIPHERAL {{ENABLE 1} {IO {PMC_MIO 40 .. 41}}}  \
  PS_ENET0_MDIO {{ENABLE 1} {IO {PS_MIO 24 .. 25}}}  \
  PS_ENET0_PERIPHERAL {{ENABLE 1} {IO {PS_MIO 0 .. 11}}}  \
  PS_ENET1_PERIPHERAL {{ENABLE 1} {IO {PS_MIO 12 .. 23}}}  \
  PS_GEN_IPI0_ENABLE 1  \
  PS_GEN_IPI0_MASTER A72  \
  PS_GEN_IPI1_ENABLE 1  \
  PS_GEN_IPI2_ENABLE 1  \
  PS_GEN_IPI3_ENABLE 1  \
  PS_GEN_IPI4_ENABLE 1  \
  PS_GEN_IPI5_ENABLE 1  \
  PS_GEN_IPI6_ENABLE 1  \
  PS_GPIO_EMIO_PERIPHERAL_ENABLE 1  \
  PS_HSDP_EGRESS_TRAFFIC JTAG  \
  PS_HSDP_INGRESS_TRAFFIC JTAG  \
  PS_HSDP_MODE None  \
  PS_I2C0_PERIPHERAL {{ENABLE 1} {IO {PMC_MIO 46 .. 47}}}  \
  PS_I2C1_PERIPHERAL {{ENABLE 1} {IO {PMC_MIO 44 .. 45}}}  \
  PS_IRQ_USAGE {{CH0 1} {CH1 1} {CH10 1} {CH11 1} {CH12 1} {CH13 1} {CH14 1} {CH15 1} {CH2 1} {CH3 1} {CH4 1} {CH5 1} {CH6 1} {CH7 1} {CH8 1} {CH9 1}}  \
  PS_MIO19 {{AUX_IO 0} {DIRECTION in} {DRIVE_STRENGTH 8mA} {OUTPUT_DATA default} {PULL disable} {SCHMITT 0} {SLEW slow} {USAGE Reserved}}  \
  PS_MIO21 {{AUX_IO 0} {DIRECTION in} {DRIVE_STRENGTH 8mA} {OUTPUT_DATA default} {PULL disable} {SCHMITT 0} {SLEW slow} {USAGE Reserved}}  \
  PS_MIO7 {{AUX_IO 0} {DIRECTION in} {DRIVE_STRENGTH 8mA} {OUTPUT_DATA default} {PULL disable} {SCHMITT 0} {SLEW slow} {USAGE Reserved}}  \
  PS_MIO9 {{AUX_IO 0} {DIRECTION in} {DRIVE_STRENGTH 8mA} {OUTPUT_DATA default} {PULL disable} {SCHMITT 0} {SLEW slow} {USAGE Reserved}}  \
  PS_M_AXI_FPD_DATA_WIDTH 32  \
  PS_M_AXI_LPD_DATA_WIDTH 32  \
  PS_NUM_FABRIC_RESETS 4  \
  PS_PCIE_RESET {{ENABLE 1}}  \
  PS_PL_CONNECTIVITY_MODE Custom  \
  PS_UART0_PERIPHERAL {{ENABLE 1} {IO {PMC_MIO 42 .. 43}}}  \
  PS_USB3_PERIPHERAL {{ENABLE 1} {IO {PMC_MIO 13 .. 25}}}  \
  PS_USE_FPD_AXI_NOC0 1  \
  PS_USE_FPD_AXI_NOC1 1  \
  PS_USE_FPD_CCI_NOC 1  \
  PS_USE_FPD_CCI_NOC0 1  \
  PS_USE_M_AXI_FPD 1  \
  PS_USE_M_AXI_LPD 1  \
  PS_USE_NOC_LPD_AXI0 1  \
  PS_USE_PMCPL_CLK0 1  \
  SMON_ALARMS Set_Alarms_On  \
  SMON_ENABLE_TEMP_AVERAGING 0  \
  SMON_TEMP_AVERAGING_SAMPLES 0 } \
  CONFIG.PS_PMC_CONFIG_APPLIED {1} \
  CONFIG.CLOCK_MODE {Custom} \
  CONFIG.PS_PL_CONNECTIVITY_MODE {Custom} \
] [get_bd_cells versal_cips_0]

# Connect the two PS to NOC AXI interfaces
set_property -dict [list \
  CONFIG.NUM_SI {8} \
  CONFIG.NUM_CLKS {8} \
  CONFIG.MC_INTERLEAVE_SIZE {1024} \
  CONFIG.MC_ADDR_BIT9 {CA7} \
  CONFIG.MC_CHAN_REGION1 {DDR_CH1} \
] [get_bd_cells axi_noc_0]
set_property -dict [list CONFIG.CATEGORY {ps_nci}] [get_bd_intf_pins /axi_noc_0/S06_AXI]
set_property -dict [list CONFIG.CATEGORY {ps_nci}] [get_bd_intf_pins /axi_noc_0/S07_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_0 { read_bw {1720} write_bw {1720} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S06_AXI]
set_property -dict [list CONFIG.CONNECTIONS {MC_1 { read_bw {1720} write_bw {1720} read_avg_burst {4} write_avg_burst {4}} }] [get_bd_intf_pins /axi_noc_0/S07_AXI]
set_property -dict [list CONFIG.ASSOCIATED_BUSIF {S06_AXI}] [get_bd_pins /axi_noc_0/aclk6]
set_property -dict [list CONFIG.ASSOCIATED_BUSIF {S07_AXI}] [get_bd_pins /axi_noc_0/aclk7]
connect_bd_intf_net [get_bd_intf_pins versal_cips_0/FPD_AXI_NOC_0] [get_bd_intf_pins axi_noc_0/S06_AXI]
connect_bd_intf_net [get_bd_intf_pins versal_cips_0/FPD_AXI_NOC_1] [get_bd_intf_pins axi_noc_0/S07_AXI]
connect_bd_net [get_bd_pins versal_cips_0/fpd_axi_noc_axi0_clk] [get_bd_pins axi_noc_0/aclk6]
connect_bd_net [get_bd_pins versal_cips_0/fpd_axi_noc_axi1_clk] [get_bd_pins axi_noc_0/aclk7]

# Procedure for creating a MIPI pipe for one camera
proc create_mipi_pipe { index } {
  set hier_obj [create_bd_cell -type hier mipi_$index]
  current_bd_instance $hier_obj
  global samples_pc
  global target
  global max_cols
  global max_rows
  
  # Create pins of the block
  create_bd_pin -dir I dphy_clk_200M
  create_bd_pin -dir I s_axi_lite_aclk
  create_bd_pin -dir I aresetn
  create_bd_pin -dir I video_aclk
  create_bd_pin -dir I video_aresetn
  create_bd_pin -dir O mipi_sub_irq
  create_bd_pin -dir O isppipeline_irq
  create_bd_pin -dir O frmbufwr_irq
  create_bd_pin -dir O iic2intc_irpt
  create_bd_pin -dir I emio_gpio
  
  # Create the interfaces of the block
  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 S_AXI_CTRL
  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 S_AXI_VIDEO
  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 m_axi_mm_video
  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:mipi_phy_rtl:1.0 mipi_phy_if
  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 IIC
  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:gpio_rtl:1.0 GPIO
  
  # Add and configure the MIPI Subsystem IP
  set mipi_csi2_rx_subsyst [ create_bd_cell -type ip -vlnv xilinx.com:ip:mipi_csi2_rx_subsystem mipi_csi2_rx_subsyst_0 ]
  set_property -dict [ list \
    CONFIG.CMN_PXL_FORMAT {RAW10} \
    CONFIG.CMN_NUM_LANES {2} \
    CONFIG.C_DPHY_LANES {2} \
    CONFIG.CMN_NUM_PIXELS $samples_pc \
    CONFIG.C_EN_CSI_V2_0 {true} \
    CONFIG.C_HS_LINE_RATE {420} \
    CONFIG.DPY_LINE_RATE {420} \
    CONFIG.SupportLevel {1} \
    CONFIG.C_HS_SETTLE_NS {158} \
  ] $mipi_csi2_rx_subsyst
 
  # Add and configure the AXI Interconnect (LPD)
  set axi_int_ctrl [ create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect axi_int_ctrl ]
  set_property -dict [list \
  CONFIG.NUM_SI {1} \
  CONFIG.NUM_MI {3} \
  ] $axi_int_ctrl
  
  # Add and configure the AXI Interconnect (HPD)
  set axi_int_video [ create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect axi_int_video ]
  set_property -dict [list \
  CONFIG.NUM_SI {1} \
  CONFIG.NUM_MI {3} \
  ] $axi_int_video
  
  # Add the ISPPipeline
  set isppipeline [ create_bd_cell -type ip -vlnv xilinx.com:hls:ISPPipeline_accel:1.0 isppipeline ]

  # Add and configure the Video Processor subsystem
  set v_proc [ create_bd_cell -type ip -vlnv xilinx.com:ip:v_proc_ss v_proc ]
  set_property -dict [ list \
    CONFIG.C_MAX_COLS $max_cols \
    CONFIG.C_MAX_ROWS $max_rows \
    CONFIG.C_ENABLE_DMA {false} \
    CONFIG.C_MAX_DATA_WIDTH {8} \
    CONFIG.C_TOPOLOGY {0} \
    CONFIG.C_SCALER_ALGORITHM {2} \
    CONFIG.C_ENABLE_CSC {true} \
    CONFIG.C_SAMPLES_PER_CLK $samples_pc \
  ] $v_proc
 
  # Add and configure the Video Frame Buffer Write
  set v_frmbuf_wr [create_bd_cell -type ip -vlnv xilinx.com:ip:v_frmbuf_wr v_frmbuf_wr]
  set_property -dict [list \
   CONFIG.C_M_AXI_MM_VIDEO_DATA_WIDTH {128} \
   CONFIG.SAMPLES_PER_CLOCK $samples_pc \
   CONFIG.AXIMM_DATA_WIDTH {128} \
   CONFIG.HAS_BGR8 {1} \
   CONFIG.HAS_Y_UV8_420 {1} \
   CONFIG.HAS_YUYV8 {1} \
   CONFIG.MAX_NR_PLANES {2} \
   CONFIG.MAX_COLS $max_cols \
   CONFIG.MAX_ROWS $max_rows \
  ] $v_frmbuf_wr
  
  # Slice for ISPPipeline reset signal
  set emio_gpio_index [expr {8+4*$index+0}]
  set reset_isppipeline [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice reset_isppipeline ]
  set_property -dict [ list \
  CONFIG.DIN_WIDTH {32} \
  CONFIG.DIN_TO $emio_gpio_index \
  CONFIG.DIN_FROM $emio_gpio_index \
  CONFIG.DOUT_WIDTH {1} \
  ] $reset_isppipeline

  connect_bd_net -net reset_isppipeline_Dout [get_bd_pins reset_isppipeline/Dout] [get_bd_pins isppipeline/ap_rst_n]
  connect_bd_net [get_bd_pins emio_gpio] [get_bd_pins reset_isppipeline/Din]

  # Slice for Vproc reset signal
  set emio_gpio_index [expr {8+4*$index+1}]
  set reset_v_proc [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice reset_v_proc ]
  set_property -dict [ list \
  CONFIG.DIN_WIDTH {32} \
  CONFIG.DIN_TO $emio_gpio_index \
  CONFIG.DIN_FROM $emio_gpio_index \
  CONFIG.DOUT_WIDTH {1} \
  ] $reset_v_proc

  connect_bd_net -net reset_v_proc_Dout [get_bd_pins reset_v_proc/Dout] [get_bd_pins v_proc/aresetn_ctrl]
  connect_bd_net [get_bd_pins emio_gpio] [get_bd_pins reset_v_proc/Din]

  # Slice for Frmbuf WR reset signal
  set emio_gpio_index [expr {8+4*$index+2}]
  set reset_frmbuf_wr [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice reset_frmbuf_wr ]
  set_property -dict [ list \
  CONFIG.DIN_WIDTH {32} \
  CONFIG.DIN_TO $emio_gpio_index \
  CONFIG.DIN_FROM $emio_gpio_index \
  CONFIG.DOUT_WIDTH {1} \
  ] $reset_frmbuf_wr

  connect_bd_net -net reset_frmbuf_wr_Dout [get_bd_pins reset_frmbuf_wr/Dout] [get_bd_pins v_frmbuf_wr/ap_rst_n]
  connect_bd_net [get_bd_pins emio_gpio] [get_bd_pins reset_frmbuf_wr/Din]

  # Add and configure AXI IIC
  set axi_iic [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_iic axi_iic_0]
  
  # Add and configure AXI GPIO
  set axi_gpio [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio axi_gpio_0]
  set_property -dict [list CONFIG.C_GPIO_WIDTH {2} CONFIG.C_ALL_OUTPUTS {1}] $axi_gpio
  
  # Connect the 200M D-PHY clock
  connect_bd_net [get_bd_pins dphy_clk_200M] [get_bd_pins mipi_csi2_rx_subsyst_0/dphy_clk_200M]
  # Connect the 300M video clock
  connect_bd_net [get_bd_pins video_aclk] [get_bd_pins mipi_csi2_rx_subsyst_0/video_aclk]
  connect_bd_net [get_bd_pins video_aclk] [get_bd_pins v_frmbuf_wr/ap_clk]
  connect_bd_net [get_bd_pins video_aclk] [get_bd_pins v_proc/aclk_axis]
  connect_bd_net [get_bd_pins video_aclk] [get_bd_pins v_proc/aclk_ctrl]
  connect_bd_net [get_bd_pins video_aclk] [get_bd_pins isppipeline/ap_clk]
  connect_bd_net [get_bd_pins video_aclk] [get_bd_pins axi_int_video/aclk]
  # Connect the 100M AXI-Lite clock
  connect_bd_net [get_bd_pins s_axi_lite_aclk] [get_bd_pins axi_int_ctrl/aclk]
  connect_bd_net [get_bd_pins s_axi_lite_aclk] [get_bd_pins mipi_csi2_rx_subsyst_0/lite_aclk]
  connect_bd_net [get_bd_pins s_axi_lite_aclk] [get_bd_pins axi_iic_0/s_axi_aclk]
  connect_bd_net [get_bd_pins s_axi_lite_aclk] [get_bd_pins axi_gpio_0/s_axi_aclk]
  # Connect the video resets
  connect_bd_net [get_bd_pins video_aresetn] [get_bd_pins axi_int_video/aresetn] -boundary_type upper
  connect_bd_net [get_bd_pins video_aresetn] [get_bd_pins mipi_csi2_rx_subsyst_0/video_aresetn]
  # Connect the AXI-Lite resets
  connect_bd_net [get_bd_pins aresetn] [get_bd_pins axi_int_ctrl/aresetn] -boundary_type upper
  connect_bd_net [get_bd_pins aresetn] [get_bd_pins mipi_csi2_rx_subsyst_0/lite_aresetn]
  connect_bd_net [get_bd_pins aresetn] [get_bd_pins axi_iic_0/s_axi_aresetn]
  connect_bd_net [get_bd_pins aresetn] [get_bd_pins axi_gpio_0/s_axi_aresetn]
  # Connect AXI Lite CTRL interfaces
  connect_bd_intf_net -boundary_type upper [get_bd_intf_pins S_AXI_CTRL] [get_bd_intf_pins axi_int_ctrl/S00_AXI]
  connect_bd_intf_net -boundary_type upper [get_bd_intf_pins axi_int_ctrl/M00_AXI] [get_bd_intf_pins mipi_csi2_rx_subsyst_0/csirxss_s_axi]
  connect_bd_intf_net -boundary_type upper [get_bd_intf_pins axi_int_ctrl/M01_AXI] [get_bd_intf_pins axi_iic_0/S_AXI]
  connect_bd_intf_net -boundary_type upper [get_bd_intf_pins axi_int_ctrl/M02_AXI] [get_bd_intf_pins axi_gpio_0/S_AXI]
  # Connect AXI Lite VIDEO interfaces
  connect_bd_intf_net -boundary_type upper [get_bd_intf_pins S_AXI_VIDEO] [get_bd_intf_pins axi_int_video/S00_AXI]
  connect_bd_intf_net -boundary_type upper [get_bd_intf_pins axi_int_video/M00_AXI] [get_bd_intf_pins isppipeline/s_axi_CTRL]
  connect_bd_intf_net -boundary_type upper [get_bd_intf_pins axi_int_video/M01_AXI] [get_bd_intf_pins v_frmbuf_wr/s_axi_CTRL]
  connect_bd_intf_net -boundary_type upper [get_bd_intf_pins axi_int_video/M02_AXI] [get_bd_intf_pins v_proc/s_axi_ctrl]
  # Connect the AXI Streaming interfaces
  connect_bd_intf_net [get_bd_intf_pins mipi_csi2_rx_subsyst_0/video_out] [get_bd_intf_pins isppipeline/s_axis_video]
  connect_bd_intf_net [get_bd_intf_pins isppipeline/m_axis_video] [get_bd_intf_pins v_proc/s_axis]
  connect_bd_intf_net [get_bd_intf_pins v_proc/m_axis] [get_bd_intf_pins v_frmbuf_wr/s_axis_video]
  # Connect the MIPI D-PHY interface
  connect_bd_intf_net -boundary_type upper [get_bd_intf_pins mipi_phy_if] [get_bd_intf_pins mipi_csi2_rx_subsyst_0/mipi_phy_if]
  # Connect the Frame Buffer MM interface
  connect_bd_intf_net [get_bd_intf_pins m_axi_mm_video] [get_bd_intf_pins v_frmbuf_wr/m_axi_mm_video]
  # Connect the I2C interface
  connect_bd_intf_net [get_bd_intf_pins IIC] [get_bd_intf_pins axi_iic_0/IIC]
  # Connect the GPIO interface
  connect_bd_intf_net [get_bd_intf_pins GPIO] [get_bd_intf_pins axi_gpio_0/GPIO]
  # Connect interrupts
  connect_bd_net [get_bd_pins mipi_sub_irq] [get_bd_pins mipi_csi2_rx_subsyst_0/csirxss_csi_irq]
  connect_bd_net [get_bd_pins isppipeline_irq] [get_bd_pins isppipeline/interrupt]
  connect_bd_net [get_bd_pins frmbufwr_irq] [get_bd_pins v_frmbuf_wr/interrupt]
  connect_bd_net [get_bd_pins iic2intc_irpt] [get_bd_pins axi_iic_0/iic2intc_irpt]
  
  current_bd_instance /
}

# Add the clock wizard
create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wizard clk_wiz_0
set_property -dict [list \
  CONFIG.USE_LOCKED {true} \
  CONFIG.USE_RESET {true} \
  CONFIG.RESET_TYPE {ACTIVE_LOW} \
  CONFIG.CLKOUT_USED {true,true,true,true,false,false,false} \
  CONFIG.CLKOUT_PORT {clk_100M,clk_150M,clk_dphy,clk_200M,clk_out5,clk_out6,clk_out7} \
  CONFIG.CLKOUT_REQUESTED_OUT_FREQUENCY {100,150,200,200.000,100.000,100.000,100.000} \
  CONFIG.CLKOUT_REQUESTED_PHASE {0.000,0.000,0.000,0.000,0.000,0.000,0.000} \
  CONFIG.CLKOUT_REQUESTED_DUTY_CYCLE {50.000,50.000,50.000,50.000,50.000,50.000,50.000} \
  CONFIG.CLKOUT_DRIVES {BUFG,BUFG,BUFG,BUFG,BUFG,BUFG,BUFG} \
  CONFIG.CLKOUT_GROUPING {Auto,Auto,Auto,Auto,Auto,Auto,Auto} \
  CONFIG.CLKOUT_DYN_PS {None,None,None,None,None,None,None} \
  CONFIG.CLKOUT_MATCHED_ROUTING {false,false,false,false,false,false,false} \
  CONFIG.CLKOUT1_DIVIDE {30.000000} \
  CONFIG.CLKOUT2_DIVIDE {20.000000} \
  CONFIG.CLKOUT3_DIVIDE {15.000000} \
  CONFIG.CLKOUT4_DIVIDE {15.000000} \
] [get_bd_cells clk_wiz_0]
connect_bd_net [get_bd_pins versal_cips_0/pl0_ref_clk] [get_bd_pins clk_wiz_0/clk_in1]
connect_bd_net [get_bd_pins versal_cips_0/pl0_resetn] [get_bd_pins clk_wiz_0/resetn]

# Proc system reset for 100M clock
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset reset_100M
connect_bd_net [get_bd_pins clk_wiz_0/clk_100M] [get_bd_pins reset_100M/slowest_sync_clk]
connect_bd_net [get_bd_pins versal_cips_0/pl0_resetn] [get_bd_pins reset_100M/ext_reset_in]
connect_bd_net [get_bd_pins clk_wiz_0/locked] [get_bd_pins reset_100M/dcm_locked]

# Proc system reset for 150M clock
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset reset_150M
connect_bd_net [get_bd_pins clk_wiz_0/clk_150M] [get_bd_pins reset_150M/slowest_sync_clk]
connect_bd_net [get_bd_pins versal_cips_0/pl0_resetn] [get_bd_pins reset_150M/ext_reset_in]
connect_bd_net [get_bd_pins clk_wiz_0/locked] [get_bd_pins reset_150M/dcm_locked]

# Proc system reset for 200M clock
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset reset_200M
connect_bd_net [get_bd_pins clk_wiz_0/clk_200M] [get_bd_pins reset_200M/slowest_sync_clk]
connect_bd_net [get_bd_pins versal_cips_0/pl0_resetn] [get_bd_pins reset_200M/ext_reset_in]
connect_bd_net [get_bd_pins clk_wiz_0/locked] [get_bd_pins reset_200M/dcm_locked]

# Extra clock wizard for the DPHY clock
create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wizard clk_wiz_1
set_property -dict [list \
  CONFIG.CLKOUT_USED {true,false,false,false,false,false,false} \
  CONFIG.CLKOUT_PORT {clk_dphy,clk_out2,clk_out3,clk_out4,clk_out5,clk_out6,clk_out7} \
  CONFIG.CLKOUT_REQUESTED_OUT_FREQUENCY {200,100.000,100.000,100.000,100.000,100.000,100.000} \
  CONFIG.CLKOUT_REQUESTED_PHASE {0.000,0.000,0.000,0.000,0.000,0.000,0.000} \
  CONFIG.CLKOUT_REQUESTED_DUTY_CYCLE {50.000,50.000,50.000,50.000,50.000,50.000,50.000} \
  CONFIG.CLKOUT_DRIVES {BUFG,BUFG,BUFG,BUFG,BUFG,BUFG,BUFG} \
  CONFIG.CLKOUT_GROUPING {Auto,Auto,Auto,Auto,Auto,Auto,Auto} \
  CONFIG.CLKOUT_DYN_PS {None,None,None,None,None,None,None} \
  CONFIG.CLKOUT_MATCHED_ROUTING {false,false,false,false,false,false,false} \
  CONFIG.CLKOUT1_DIVIDE {15.000000} \
] [get_bd_cells clk_wiz_1]
connect_bd_net [get_bd_pins versal_cips_0/pl0_ref_clk] [get_bd_pins clk_wiz_1/clk_in1]

# Add smartconnect to M_AXI_FPD interface
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect axi_smc_fpd
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {3} CONFIG.NUM_CLKS {3}] [get_bd_cells axi_smc_fpd]
connect_bd_net [get_bd_pins clk_wiz_0/clk_100M] [get_bd_pins axi_smc_fpd/aclk]
connect_bd_net [get_bd_pins clk_wiz_0/clk_150M] [get_bd_pins axi_smc_fpd/aclk1]
connect_bd_net [get_bd_pins clk_wiz_0/clk_200M] [get_bd_pins axi_smc_fpd/aclk2]
connect_bd_net [get_bd_pins clk_wiz_0/clk_100M] [get_bd_pins versal_cips_0/m_axi_fpd_aclk]
connect_bd_intf_net [get_bd_intf_pins versal_cips_0/M_AXI_FPD] [get_bd_intf_pins axi_smc_fpd/S00_AXI]
connect_bd_net [get_bd_pins reset_100M/interconnect_aresetn] [get_bd_pins axi_smc_fpd/aresetn]

# Add processor system reset for the PL0_REF_CLK 100MHz
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset reset_pl0_ref_clk
connect_bd_net [get_bd_pins versal_cips_0/pl0_ref_clk] [get_bd_pins reset_pl0_ref_clk/slowest_sync_clk]
connect_bd_net [get_bd_pins versal_cips_0/pl0_resetn] [get_bd_pins reset_pl0_ref_clk/ext_reset_in]

# Add smartconnect to M_AXI_LPD interface
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect axi_smc_lpd
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1} CONFIG.NUM_CLKS {1}] [get_bd_cells axi_smc_lpd]
connect_bd_net [get_bd_pins clk_wiz_0/clk_150M] [get_bd_pins axi_smc_lpd/aclk]
connect_bd_net [get_bd_pins reset_100M/interconnect_aresetn] [get_bd_pins axi_smc_lpd/aresetn]
connect_bd_net [get_bd_pins clk_wiz_0/clk_150M] [get_bd_pins versal_cips_0/m_axi_lpd_aclk]
connect_bd_intf_net [get_bd_intf_pins versal_cips_0/M_AXI_LPD] [get_bd_intf_pins axi_smc_lpd/S00_AXI]

# Add AXI Verification IP
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_vip axi_vip_0
set_property -dict [list CONFIG.INTERFACE_MODE {SLAVE}] [get_bd_cells axi_vip_0]
connect_bd_net [get_bd_pins clk_wiz_0/clk_150M] [get_bd_pins axi_vip_0/aclk]
connect_bd_net [get_bd_pins reset_150M/interconnect_aresetn] [get_bd_pins axi_vip_0/aresetn]
connect_bd_intf_net [get_bd_intf_pins axi_smc_lpd/M00_AXI] [get_bd_intf_pins axi_vip_0/S_AXI]

# Add AXI Interrupt controller
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_intc axi_intc_0
lappend m_axi_fpd_ports "axi_intc_0/s_axi"
connect_bd_net [get_bd_pins clk_wiz_0/clk_150M] [get_bd_pins axi_intc_0/s_axi_aclk]
connect_bd_net [get_bd_pins reset_150M/peripheral_aresetn] [get_bd_pins axi_intc_0/s_axi_aresetn]
connect_bd_net [get_bd_pins axi_intc_0/irq] [get_bd_pins versal_cips_0/pl_ps_irq15]

# Add the MIPI pipes
set cam_index 0
foreach i $cams {
  # Create the MIPI pipe block
  create_mipi_pipe $i
  # Connect clocks
  set bank [lindex $bank_index $i]
  connect_bd_net [get_bd_pins clk_wiz_${bank}/clk_dphy] [get_bd_pins mipi_$i/dphy_clk_200M]
  connect_bd_net [get_bd_pins clk_wiz_0/clk_100M] [get_bd_pins mipi_$i/s_axi_lite_aclk]
  connect_bd_net [get_bd_pins clk_wiz_0/clk_200M] [get_bd_pins mipi_$i/video_aclk]
  # Connect resets
  connect_bd_net [get_bd_pins reset_100M/peripheral_aresetn] [get_bd_pins mipi_$i/aresetn]
  connect_bd_net [get_bd_pins reset_200M/peripheral_aresetn] [get_bd_pins mipi_$i/video_aresetn]
  # Connect EMIO GPIO
  connect_bd_net [get_bd_pins versal_cips_0/LPD_GPIO_o] [get_bd_pins mipi_$i/emio_gpio]
  # Add interrupts to the interrupt list to be connected later
  lappend intr_list "mipi_$i/mipi_sub_irq"
  lappend intr_list "mipi_$i/isppipeline_irq"
  lappend intr_list "mipi_$i/frmbufwr_irq"
  lappend intr_list "mipi_$i/iic2intc_irpt"
  
  # AXI Lite interfaces to be connected later
  lappend m_axi_fpd_ports "mipi_$i/S_AXI_CTRL"
  lappend m_axi_fpd_ports "mipi_$i/S_AXI_VIDEO"
  # Connect the MIPI D-Phy interface
  create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:mipi_phy_rtl:1.0 mipi_phy_if_$i
  connect_bd_intf_net [get_bd_intf_ports mipi_phy_if_$i] -boundary_type upper [get_bd_intf_pins mipi_$i/mipi_phy_if]
  # Connect the I2C interface
  create_bd_intf_port -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 iic_$i
  connect_bd_intf_net [get_bd_intf_ports iic_$i] [get_bd_intf_pins mipi_$i/IIC]
  # Connect the GPIO interface
  create_bd_intf_port -mode Master -vlnv xilinx.com:interface:gpio_rtl:1.0 gpio_$i
  connect_bd_intf_net [get_bd_intf_ports gpio_$i] [get_bd_intf_pins mipi_$i/GPIO]
  # Connect the AXI MM interfaces of the Frame Buffers
  lappend noc_ports [list "mipi_$i/m_axi_mm_video" "clk_wiz_0/clk_200M"]
  # Increment the cam index
  set cam_index [expr {$cam_index+1}]
}

##########################################################
# Display pipe
##########################################################

# Hierarchical cell: gt_refclk1
proc create_hier_cell_gt_refclk1 { parentCell nameHier } {

  variable script_folder

  if { $parentCell eq "" || $nameHier eq "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2092 -severity "ERROR" "create_hier_cell_gt_refclk1() - Empty argument(s)!"}
     return
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2090 -severity "ERROR" "Unable to find parent cell <$parentCell>!"}
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2091 -severity "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj

  # Create cell and set as current instance
  set hier_obj [create_bd_cell -type hier $nameHier]
  current_bd_instance $hier_obj

  # Create interface pins

  # Create pins
  create_bd_pin -dir I -from 0 -to 0 -type clk CLK_N_IN
  create_bd_pin -dir I -from 0 -to 0 -type clk CLK_P_IN
  create_bd_pin -dir O -from 0 -to 0 -type clk O
  create_bd_pin -dir O -from 0 -to 0 -type clk ODIV2

  # Create instance: dru_ibufds_gt_odiv2, and set properties
  set dru_ibufds_gt_odiv2 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf dru_ibufds_gt_odiv2 ]
  set_property -dict [ list \
   CONFIG.C_BUF_TYPE {BUFG_GT} \
 ] $dru_ibufds_gt_odiv2

  # Create instance: gt_refclk_buf, and set properties
  set gt_refclk_buf [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf gt_refclk_buf ]
  set_property -dict [ list \
   CONFIG.C_BUF_TYPE {IBUFDSGTE} \
 ] $gt_refclk_buf

  # Create instance: vcc_const0, and set properties
  set vcc_const0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant vcc_const0 ]
  set_property -dict [ list \
   CONFIG.CONST_VAL {1} \
 ] $vcc_const0

  # Create port connections
  connect_bd_net -net HDMI_RX_CLK_N_IN_1 [get_bd_pins CLK_N_IN] [get_bd_pins gt_refclk_buf/IBUF_DS_N]
  connect_bd_net -net HDMI_RX_CLK_P_IN_1 [get_bd_pins CLK_P_IN] [get_bd_pins gt_refclk_buf/IBUF_DS_P]
  connect_bd_net -net dru_ibufds_gt_odiv2_BUFG_GT_O [get_bd_pins ODIV2] [get_bd_pins dru_ibufds_gt_odiv2/BUFG_GT_O]
  connect_bd_net -net gt_refclk_buf_IBUF_OUT [get_bd_pins O] [get_bd_pins gt_refclk_buf/IBUF_OUT]
  connect_bd_net -net net_gt_refclk_buf_IBUF_DS_ODIV2 [get_bd_pins dru_ibufds_gt_odiv2/BUFG_GT_I] [get_bd_pins gt_refclk_buf/IBUF_DS_ODIV2]
  connect_bd_net -net net_vcc_const0_dout [get_bd_pins dru_ibufds_gt_odiv2/BUFG_GT_CE] [get_bd_pins vcc_const0/dout]

  # Restore current instance
  current_bd_instance $oldCurInst
}

# Hierarchical cell: GT_Quad_and_Clk
proc create_hier_cell_GT_Quad_and_Clk { parentCell nameHier } {

  variable script_folder

  if { $parentCell eq "" || $nameHier eq "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2092 -severity "ERROR" "create_hier_cell_GT_Quad_and_Clk() - Empty argument(s)!"}
     return
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2090 -severity "ERROR" "Unable to find parent cell <$parentCell>!"}
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2091 -severity "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj

  # Create cell and set as current instance
  set hier_obj [create_bd_cell -type hier $nameHier]
  current_bd_instance $hier_obj

  # Create interface pins
  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:gt_channel_debug_rtl:1.0 CH0_DEBUG

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:gt_channel_debug_rtl:1.0 CH1_DEBUG

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:gt_channel_debug_rtl:1.0 CH2_DEBUG

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:gt_channel_debug_rtl:1.0 CH3_DEBUG

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:gt_debug_rtl:1.0 GT_DEBUG

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:gt_tx_interface_rtl:1.0 TX0_GT_IP_Interface

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:gt_tx_interface_rtl:1.0 TX1_GT_IP_Interface

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:gt_tx_interface_rtl:1.0 TX2_GT_IP_Interface

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:gt_tx_interface_rtl:1.0 TX3_GT_IP_Interface


  # Create pins
  create_bd_pin -dir I -type clk GT_REFCLK1
  create_bd_pin -dir I -from 3 -to 0 RX_DATA_IN_rxn
  create_bd_pin -dir I -from 3 -to 0 RX_DATA_IN_rxp
  create_bd_pin -dir O -from 3 -to 0 TX_DATA_OUT_txn
  create_bd_pin -dir O -from 3 -to 0 TX_DATA_OUT_txp
  create_bd_pin -dir I altclk
  create_bd_pin -dir O ch0_iloresetdone
  create_bd_pin -dir O ch1_iloresetdone
  create_bd_pin -dir O ch2_iloresetdone
  create_bd_pin -dir O ch3_iloresetdone
  create_bd_pin -dir O gtpowergood
  create_bd_pin -dir O hsclk0_lcplllock
  create_bd_pin -dir I -type rst hsclk0_lcpllreset
  create_bd_pin -dir O hsclk1_lcplllock
  create_bd_pin -dir I -type rst hsclk1_lcpllreset
  create_bd_pin -dir O -type gt_usrclk tx_usrclk

  # Create instance: bufg_gt_tx, and set properties
  set bufg_gt_tx [ create_bd_cell -type ip -vlnv xilinx.com:ip:bufg_gt bufg_gt_tx ]

  # Create instance: gt_quad_base_1, and set properties
  set gt_quad_base_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:gt_quad_base gt_quad_base_1 ]
  set_property -dict [ list \
   CONFIG.REFCLK_STRING {HSCLK0_LCPLLGTREFCLK1 refclk_PROT0_R1_multiple_ext_freq HSCLK1_LCPLLGTREFCLK1 refclk_PROT0_R1_multiple_ext_freq} \
 ] $gt_quad_base_1

  # Create instance: xlconstant_0, and set properties
  set xlconstant_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant xlconstant_0 ]
  set_property -dict [ list \
   CONFIG.CONST_VAL {0} \
 ] $xlconstant_0

  # Create interface connections
  connect_bd_intf_net -intf_net CH0_DEBUG_1 [get_bd_intf_pins CH0_DEBUG] [get_bd_intf_pins gt_quad_base_1/CH0_DEBUG]
  connect_bd_intf_net -intf_net CH1_DEBUG_1 [get_bd_intf_pins CH1_DEBUG] [get_bd_intf_pins gt_quad_base_1/CH1_DEBUG]
  connect_bd_intf_net -intf_net CH2_DEBUG_1 [get_bd_intf_pins CH2_DEBUG] [get_bd_intf_pins gt_quad_base_1/CH2_DEBUG]
  connect_bd_intf_net -intf_net CH3_DEBUG_1 [get_bd_intf_pins CH3_DEBUG] [get_bd_intf_pins gt_quad_base_1/CH3_DEBUG]
  connect_bd_intf_net -intf_net GT_DEBUG_1 [get_bd_intf_pins GT_DEBUG] [get_bd_intf_pins gt_quad_base_1/GT_DEBUG]
  connect_bd_intf_net -intf_net TX0_GT_IP_Interface_1 [get_bd_intf_pins TX0_GT_IP_Interface] [get_bd_intf_pins gt_quad_base_1/TX0_GT_IP_Interface]
  connect_bd_intf_net -intf_net TX1_GT_IP_Interface_1 [get_bd_intf_pins TX1_GT_IP_Interface] [get_bd_intf_pins gt_quad_base_1/TX1_GT_IP_Interface]
  connect_bd_intf_net -intf_net TX2_GT_IP_Interface_1 [get_bd_intf_pins TX2_GT_IP_Interface] [get_bd_intf_pins gt_quad_base_1/TX2_GT_IP_Interface]
  connect_bd_intf_net -intf_net TX3_GT_IP_Interface_1 [get_bd_intf_pins TX3_GT_IP_Interface] [get_bd_intf_pins gt_quad_base_1/TX3_GT_IP_Interface]

  # Create port connections
  connect_bd_net -net GT_REFCLK1_1 [get_bd_pins GT_REFCLK1] [get_bd_pins gt_quad_base_1/GT_REFCLK0]
  connect_bd_net -net RX_DATA_IN_rxn [get_bd_pins RX_DATA_IN_rxn] [get_bd_pins gt_quad_base_1/rxn]
  connect_bd_net -net RX_DATA_IN_rxp [get_bd_pins RX_DATA_IN_rxp] [get_bd_pins gt_quad_base_1/rxp]
  connect_bd_net -net altclk_1 [get_bd_pins altclk] [get_bd_pins gt_quad_base_1/altclk] -boundary_type upper
  connect_bd_net -net altclk_1 [get_bd_pins altclk] [get_bd_pins gt_quad_base_1/apb3clk] -boundary_type upper
  connect_bd_net -net bufg_gt_1_usrclk [get_bd_pins tx_usrclk] [get_bd_pins bufg_gt_tx/usrclk] -boundary_type upper
  connect_bd_net -net bufg_gt_1_usrclk [get_bd_pins tx_usrclk] [get_bd_pins gt_quad_base_1/ch0_txusrclk] -boundary_type upper
  connect_bd_net -net bufg_gt_1_usrclk [get_bd_pins tx_usrclk] [get_bd_pins gt_quad_base_1/ch1_txusrclk] -boundary_type upper
  connect_bd_net -net bufg_gt_1_usrclk [get_bd_pins tx_usrclk] [get_bd_pins gt_quad_base_1/ch2_txusrclk] -boundary_type upper
  connect_bd_net -net bufg_gt_1_usrclk [get_bd_pins tx_usrclk] [get_bd_pins gt_quad_base_1/ch3_txusrclk] -boundary_type upper
  connect_bd_net -net gt_quad_base_1_ch0_iloresetdone [get_bd_pins ch0_iloresetdone] [get_bd_pins gt_quad_base_1/ch0_iloresetdone]
  connect_bd_net -net gt_quad_base_1_ch0_txoutclk [get_bd_pins bufg_gt_tx/outclk] [get_bd_pins gt_quad_base_1/ch0_txoutclk]
  connect_bd_net -net gt_quad_base_1_ch1_iloresetdone [get_bd_pins ch1_iloresetdone] [get_bd_pins gt_quad_base_1/ch1_iloresetdone]
  connect_bd_net -net gt_quad_base_1_ch2_iloresetdone [get_bd_pins ch2_iloresetdone] [get_bd_pins gt_quad_base_1/ch2_iloresetdone]
  connect_bd_net -net gt_quad_base_1_ch3_iloresetdone [get_bd_pins ch3_iloresetdone] [get_bd_pins gt_quad_base_1/ch3_iloresetdone]
  connect_bd_net -net gt_quad_base_1_gtpowergood [get_bd_pins gtpowergood] [get_bd_pins gt_quad_base_1/gtpowergood]
  connect_bd_net -net gt_quad_base_1_hsclk0_lcplllock [get_bd_pins hsclk0_lcplllock] [get_bd_pins gt_quad_base_1/hsclk0_lcplllock]
  connect_bd_net -net gt_quad_base_1_hsclk1_lcplllock [get_bd_pins hsclk1_lcplllock] [get_bd_pins gt_quad_base_1/hsclk1_lcplllock]
  connect_bd_net -net gt_quad_base_1_txn [get_bd_pins TX_DATA_OUT_txn] [get_bd_pins gt_quad_base_1/txn]
  connect_bd_net -net gt_quad_base_1_txp [get_bd_pins TX_DATA_OUT_txp] [get_bd_pins gt_quad_base_1/txp]
  connect_bd_net -net hsclk0_lcpllreset_1 [get_bd_pins hsclk0_lcpllreset] [get_bd_pins gt_quad_base_1/hsclk0_lcpllreset]
  connect_bd_net -net hsclk1_lcpllreset_1 [get_bd_pins hsclk1_lcpllreset] [get_bd_pins gt_quad_base_1/hsclk1_lcpllreset]
  connect_bd_net -net xlconstant_0_dout [get_bd_pins gt_quad_base_1/ch0_rxusrclk] [get_bd_pins gt_quad_base_1/ch1_rxusrclk] -boundary_type upper
  connect_bd_net -net xlconstant_0_dout [get_bd_pins gt_quad_base_1/ch0_rxusrclk] [get_bd_pins gt_quad_base_1/ch2_rxusrclk] -boundary_type upper
  connect_bd_net -net xlconstant_0_dout [get_bd_pins gt_quad_base_1/ch0_rxusrclk] [get_bd_pins gt_quad_base_1/ch3_rxusrclk] -boundary_type upper
  connect_bd_net -net xlconstant_0_dout [get_bd_pins gt_quad_base_1/ch0_rxusrclk] [get_bd_pins xlconstant_0/dout] -boundary_type upper

  # Restore current instance
  current_bd_instance $oldCurInst
}

# Hierarchical cell: hdmi_tx_phy
proc create_hier_cell_hdmi_tx_phy { parentCell nameHier } {

  variable script_folder

  if { $parentCell eq "" || $nameHier eq "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2092 -severity "ERROR" "create_hier_cell_hdmi_tx_phy() - Empty argument(s)!"}
     return
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2090 -severity "ERROR" "Unable to find parent cell <$parentCell>!"}
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2091 -severity "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj

  # Create cell and set as current instance
  set hier_obj [create_bd_cell -type hier $nameHier]
  current_bd_instance $hier_obj

  # Create interface pins
  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 HDMI_CTL_IIC

  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:axis_rtl:1.0 M_AXIS

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 S_AXI

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 axi4lite

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:axis_rtl:1.0 s_axis_video

  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:axis_rtl:1.0 status_sb_tx

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:axis_rtl:1.0 tx_axi4s_ch0

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:axis_rtl:1.0 tx_axi4s_ch1

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:axis_rtl:1.0 tx_axi4s_ch2


  # Create pins
  create_bd_pin -dir I -type rst ARESETN
  create_bd_pin -dir I IDT_8T49N241_LOL_IN
  create_bd_pin -dir I -from 3 -to 0 RX_DATA_IN_rxn
  create_bd_pin -dir I -from 3 -to 0 RX_DATA_IN_rxp
  create_bd_pin -dir O -from 3 -to 0 TX_DATA_OUT_txn
  create_bd_pin -dir O -from 3 -to 0 TX_DATA_OUT_txp
  create_bd_pin -dir O -from 0 -to 0 -type rst TX_EN_OUT
  create_bd_pin -dir I -from 0 -to 0 -type clk TX_REFCLK_N_IN
  create_bd_pin -dir I -from 0 -to 0 -type clk TX_REFCLK_P_IN
  create_bd_pin -dir I -type clk aclk
  create_bd_pin -dir I altclk
  create_bd_pin -dir I -type rst aresetn1
  create_bd_pin -dir O -type intr iic2intc_irpt
  create_bd_pin -dir O -type intr irq
  create_bd_pin -dir O -type clk tx_tmds_clk
  create_bd_pin -dir O -type gt_usrclk tx_usrclk
  create_bd_pin -dir O -type clk tx_video_clk

  # Create instance: GT_Quad_and_Clk
  create_hier_cell_GT_Quad_and_Clk $hier_obj GT_Quad_and_Clk

  # Create instance: fmch_axi_iic, and set properties
  set fmch_axi_iic [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_iic fmch_axi_iic ]

  # Create instance: gt_refclk1
  create_hier_cell_gt_refclk1 $hier_obj gt_refclk1

  # Create instance: hdmi_gt_controller_1, and set properties
  set hdmi_gt_controller_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:hdmi_gt_controller hdmi_gt_controller_1 ]
  set_property -dict [ list \
   CONFIG.C_GT_DEBUG_PORT_EN {true} \
   CONFIG.C_GT_DIRECTION {SIMPLEX_TX} \
   CONFIG.C_INPUT_PIXELS_PER_CLOCK {4} \
   CONFIG.C_NIDRU {false} \
   CONFIG.C_NIDRU_REFCLK_SEL {2} \
   CONFIG.C_RX_PLL_SELECTION {8} \
   CONFIG.C_RX_REFCLK_SEL {0} \
   CONFIG.C_Rx_Protocol {None} \
   CONFIG.C_TX_PLL_SELECTION {7} \
   CONFIG.C_TX_REFCLK_SEL {1} \
   CONFIG.C_Tx_No_Of_Channels {4} \
   CONFIG.C_Tx_Protocol {HDMI} \
   CONFIG.C_Txrefclk_Rdy_Invert {true} \
   CONFIG.C_Use_GT_CH4_HDMI {true} \
   CONFIG.C_Use_Oddr_for_Tmds_Clkout {false} \
   CONFIG.C_vid_phy_rx_axi4s_ch_INT_TDATA_WIDTH {40} \
   CONFIG.C_vid_phy_rx_axi4s_ch_TDATA_WIDTH {40} \
   CONFIG.C_vid_phy_tx_axi4s_ch_INT_TDATA_WIDTH {40} \
   CONFIG.C_vid_phy_tx_axi4s_ch_TDATA_WIDTH {40} \
   CONFIG.Transceiver_Width {4} \
   CONFIG.check_refclk_selection {0} \
 ] $hdmi_gt_controller_1

  # Create instance: tx_video_axis_reg_slice, and set properties
  set tx_video_axis_reg_slice [ create_bd_cell -type ip -vlnv xilinx.com:ip:axis_register_slice tx_video_axis_reg_slice ]

  # Create instance: vcc_const, and set properties
  set vcc_const [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant vcc_const ]
  set_property -dict [ list \
   CONFIG.CONST_VAL {1} \
 ] $vcc_const

  # Create interface connections
  connect_bd_intf_net -intf_net Conn1 [get_bd_intf_pins M_AXIS] [get_bd_intf_pins tx_video_axis_reg_slice/M_AXIS]
  connect_bd_intf_net -intf_net Conn2 [get_bd_intf_pins tx_axi4s_ch0] [get_bd_intf_pins hdmi_gt_controller_1/tx_axi4s_ch0]
  connect_bd_intf_net -intf_net Conn3 [get_bd_intf_pins tx_axi4s_ch1] [get_bd_intf_pins hdmi_gt_controller_1/tx_axi4s_ch1]
  connect_bd_intf_net -intf_net Conn4 [get_bd_intf_pins tx_axi4s_ch2] [get_bd_intf_pins hdmi_gt_controller_1/tx_axi4s_ch2]
  connect_bd_intf_net -intf_net Conn5 [get_bd_intf_pins status_sb_tx] [get_bd_intf_pins hdmi_gt_controller_1/status_sb_tx]
  connect_bd_intf_net -intf_net hdmi_gt_controller_1_ch0_debug [get_bd_intf_pins GT_Quad_and_Clk/CH0_DEBUG] [get_bd_intf_pins hdmi_gt_controller_1/ch0_debug]
  connect_bd_intf_net -intf_net hdmi_gt_controller_1_ch1_debug [get_bd_intf_pins GT_Quad_and_Clk/CH1_DEBUG] [get_bd_intf_pins hdmi_gt_controller_1/ch1_debug]
  connect_bd_intf_net -intf_net hdmi_gt_controller_1_ch2_debug [get_bd_intf_pins GT_Quad_and_Clk/CH2_DEBUG] [get_bd_intf_pins hdmi_gt_controller_1/ch2_debug]
  connect_bd_intf_net -intf_net hdmi_gt_controller_1_ch3_debug [get_bd_intf_pins GT_Quad_and_Clk/CH3_DEBUG] [get_bd_intf_pins hdmi_gt_controller_1/ch3_debug]
  connect_bd_intf_net -intf_net hdmi_gt_controller_1_gt_debug [get_bd_intf_pins GT_Quad_and_Clk/GT_DEBUG] [get_bd_intf_pins hdmi_gt_controller_1/gt_debug]
  connect_bd_intf_net -intf_net hdmi_gt_controller_1_gt_tx0 [get_bd_intf_pins GT_Quad_and_Clk/TX0_GT_IP_Interface] [get_bd_intf_pins hdmi_gt_controller_1/gt_tx0]
  connect_bd_intf_net -intf_net hdmi_gt_controller_1_gt_tx1 [get_bd_intf_pins GT_Quad_and_Clk/TX1_GT_IP_Interface] [get_bd_intf_pins hdmi_gt_controller_1/gt_tx1]
  connect_bd_intf_net -intf_net hdmi_gt_controller_1_gt_tx2 [get_bd_intf_pins GT_Quad_and_Clk/TX2_GT_IP_Interface] [get_bd_intf_pins hdmi_gt_controller_1/gt_tx2]
  connect_bd_intf_net -intf_net hdmi_gt_controller_1_gt_tx3 [get_bd_intf_pins GT_Quad_and_Clk/TX3_GT_IP_Interface] [get_bd_intf_pins hdmi_gt_controller_1/gt_tx3]
  connect_bd_intf_net -intf_net hdmi_tx_pipe_HDMI_CTL_IIC [get_bd_intf_pins HDMI_CTL_IIC] [get_bd_intf_pins fmch_axi_iic/IIC]
  connect_bd_intf_net -intf_net s_axis_video_1 [get_bd_intf_pins s_axis_video] [get_bd_intf_pins tx_video_axis_reg_slice/S_AXIS]
  connect_bd_intf_net -intf_net smartconnect_100mhz_M00_AXI [get_bd_intf_pins axi4lite] [get_bd_intf_pins hdmi_gt_controller_1/axi4lite]
  connect_bd_intf_net -intf_net smartconnect_100mhz_M04_AXI [get_bd_intf_pins S_AXI] [get_bd_intf_pins fmch_axi_iic/S_AXI]

  # Create port connections
  connect_bd_net -net GT_Quad_and_Clk_ch0_iloresetdone [get_bd_pins GT_Quad_and_Clk/ch0_iloresetdone] [get_bd_pins hdmi_gt_controller_1/gt_ch0_ilo_resetdone]
  connect_bd_net -net GT_Quad_and_Clk_ch1_iloresetdone [get_bd_pins GT_Quad_and_Clk/ch1_iloresetdone] [get_bd_pins hdmi_gt_controller_1/gt_ch1_ilo_resetdone]
  connect_bd_net -net GT_Quad_and_Clk_ch2_iloresetdone [get_bd_pins GT_Quad_and_Clk/ch2_iloresetdone] [get_bd_pins hdmi_gt_controller_1/gt_ch2_ilo_resetdone]
  connect_bd_net -net GT_Quad_and_Clk_ch3_iloresetdone [get_bd_pins GT_Quad_and_Clk/ch3_iloresetdone] [get_bd_pins hdmi_gt_controller_1/gt_ch3_ilo_resetdone]
  connect_bd_net -net GT_Quad_and_Clk_gtpowergood [get_bd_pins GT_Quad_and_Clk/gtpowergood] [get_bd_pins hdmi_gt_controller_1/gtpowergood]
  connect_bd_net -net GT_Quad_and_Clk_hsclk0_lcplllock [get_bd_pins GT_Quad_and_Clk/hsclk0_lcplllock] [get_bd_pins hdmi_gt_controller_1/gt_lcpll0_lock]
  connect_bd_net -net GT_Quad_and_Clk_hsclk1_lcplllock [get_bd_pins GT_Quad_and_Clk/hsclk1_lcplllock] [get_bd_pins hdmi_gt_controller_1/gt_lcpll1_lock]
  connect_bd_net -net GT_Quad_and_Clk_txn_0 [get_bd_pins TX_DATA_OUT_txn] [get_bd_pins GT_Quad_and_Clk/TX_DATA_OUT_txn]
  connect_bd_net -net GT_Quad_and_Clk_txp_0 [get_bd_pins TX_DATA_OUT_txp] [get_bd_pins GT_Quad_and_Clk/TX_DATA_OUT_txp]
  connect_bd_net -net RX_DATA_IN_rxn [get_bd_pins RX_DATA_IN_rxn] [get_bd_pins GT_Quad_and_Clk/RX_DATA_IN_rxn]
  connect_bd_net -net RX_DATA_IN_rxp [get_bd_pins RX_DATA_IN_rxp] [get_bd_pins GT_Quad_and_Clk/RX_DATA_IN_rxp]
  connect_bd_net -net TX_REFCLK_N_IN_1 [get_bd_pins TX_REFCLK_N_IN] [get_bd_pins gt_refclk1/CLK_N_IN]
  connect_bd_net -net TX_REFCLK_P_IN_1 [get_bd_pins TX_REFCLK_P_IN] [get_bd_pins gt_refclk1/CLK_P_IN]
  connect_bd_net -net bufg_gt_1_usrclk [get_bd_pins tx_usrclk] [get_bd_pins GT_Quad_and_Clk/tx_usrclk] -boundary_type upper
  connect_bd_net -net bufg_gt_1_usrclk [get_bd_pins tx_usrclk] [get_bd_pins hdmi_gt_controller_1/gt_txusrclk] -boundary_type upper
  connect_bd_net -net bufg_gt_1_usrclk [get_bd_pins tx_usrclk] [get_bd_pins hdmi_gt_controller_1/tx_axi4s_aclk] -boundary_type upper
  connect_bd_net -net fmch_axi_iic_iic2intc_irpt [get_bd_pins iic2intc_irpt] [get_bd_pins fmch_axi_iic/iic2intc_irpt]
  connect_bd_net -net gt_refclk1_O [get_bd_pins GT_Quad_and_Clk/GT_REFCLK1] [get_bd_pins gt_refclk1/O]
  connect_bd_net -net gt_refclk1_ODIV2 [get_bd_pins gt_refclk1/ODIV2] [get_bd_pins hdmi_gt_controller_1/gt_refclk1_odiv2]
  connect_bd_net -net hdmi_gt_controller_1_gt_lcpll0_reset [get_bd_pins GT_Quad_and_Clk/hsclk0_lcpllreset] [get_bd_pins hdmi_gt_controller_1/gt_lcpll0_reset]
  connect_bd_net -net hdmi_gt_controller_1_gt_lcpll1_reset [get_bd_pins GT_Quad_and_Clk/hsclk1_lcpllreset] [get_bd_pins hdmi_gt_controller_1/gt_lcpll1_reset]
  connect_bd_net -net hdmi_gt_controller_1_irq [get_bd_pins irq] [get_bd_pins hdmi_gt_controller_1/irq]
  connect_bd_net -net hdmi_gt_controller_1_tx_tmds_clk [get_bd_pins tx_tmds_clk] [get_bd_pins hdmi_gt_controller_1/tx_tmds_clk]
  connect_bd_net -net hdmi_gt_controller_1_tx_video_clk [get_bd_pins tx_video_clk] [get_bd_pins hdmi_gt_controller_1/tx_video_clk]
  connect_bd_net -net net_bdry_in_SI5324_LOL_IN [get_bd_pins IDT_8T49N241_LOL_IN] [get_bd_pins hdmi_gt_controller_1/tx_refclk_rdy]
  connect_bd_net -net net_mb_ss_0_clk_out2 [get_bd_pins aclk] [get_bd_pins tx_video_axis_reg_slice/aclk]
  connect_bd_net -net net_mb_ss_0_dcm_locked [get_bd_pins aresetn1] [get_bd_pins tx_video_axis_reg_slice/aresetn]
  connect_bd_net -net net_mb_ss_0_peripheral_aresetn [get_bd_pins ARESETN] [get_bd_pins fmch_axi_iic/s_axi_aresetn] -boundary_type upper
  connect_bd_net -net net_mb_ss_0_peripheral_aresetn [get_bd_pins ARESETN] [get_bd_pins hdmi_gt_controller_1/axi4lite_aresetn] -boundary_type upper
  connect_bd_net -net net_mb_ss_0_peripheral_aresetn [get_bd_pins ARESETN] [get_bd_pins hdmi_gt_controller_1/sb_aresetn] -boundary_type upper
  connect_bd_net -net net_mb_ss_0_s_axi_aclk [get_bd_pins altclk] [get_bd_pins GT_Quad_and_Clk/altclk] -boundary_type upper
  connect_bd_net -net net_mb_ss_0_s_axi_aclk [get_bd_pins altclk] [get_bd_pins fmch_axi_iic/s_axi_aclk] -boundary_type upper
  connect_bd_net -net net_mb_ss_0_s_axi_aclk [get_bd_pins altclk] [get_bd_pins hdmi_gt_controller_1/apb_clk] -boundary_type upper
  connect_bd_net -net net_mb_ss_0_s_axi_aclk [get_bd_pins altclk] [get_bd_pins hdmi_gt_controller_1/axi4lite_aclk] -boundary_type upper
  connect_bd_net -net net_mb_ss_0_s_axi_aclk [get_bd_pins altclk] [get_bd_pins hdmi_gt_controller_1/sb_aclk] -boundary_type upper
  connect_bd_net -net net_vcc_const_dout [get_bd_pins TX_EN_OUT] [get_bd_pins hdmi_gt_controller_1/tx_axi4s_aresetn] -boundary_type upper
  connect_bd_net -net net_vcc_const_dout [get_bd_pins TX_EN_OUT] [get_bd_pins vcc_const/dout] -boundary_type upper

  # Restore current instance
  current_bd_instance $oldCurInst
}

# Hierarchical cell: display_pipe
proc create_hier_cell_display_pipe { parentCell nameHier } {

  variable script_folder

  if { $parentCell eq "" || $nameHier eq "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2092 -severity "ERROR" "create_hier_cell_display_pipe() - Empty argument(s)!"}
     return
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2090 -severity "ERROR" "Unable to find parent cell <$parentCell>!"}
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2091 -severity "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj

  # Create cell and set as current instance
  set hier_obj [create_bd_cell -type hier $nameHier]
  current_bd_instance $hier_obj

  # Create interface pins
  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:axis_rtl:1.0 AUDIO_IN

  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 HDMI_CTL_IIC

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 S_AXI

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 S_AXI_CPU_IN

  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 TX_DDC_OUT

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 axi4lite

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 s_axi_ctrl_vmix

  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 vmix_mm_axi_vid_rd_0

  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 vmix_mm_axi_vid_rd_1

  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 vmix_mm_axi_vid_rd_2


  # Create pins
  create_bd_pin -dir I -type rst ARESETN
  create_bd_pin -dir I -from 31 -to 0 Din
  create_bd_pin -dir I IDT_8T49N241_LOL_IN
  create_bd_pin -dir I -from 3 -to 0 RX_DATA_IN_rxn
  create_bd_pin -dir I -from 3 -to 0 RX_DATA_IN_rxp
  create_bd_pin -dir O -from 3 -to 0 TX_DATA_OUT_txn
  create_bd_pin -dir O -from 3 -to 0 TX_DATA_OUT_txp
  create_bd_pin -dir O -from 0 -to 0 -type rst TX_EN_OUT
  create_bd_pin -dir I TX_HPD_IN
  create_bd_pin -dir I -from 0 -to 0 -type clk TX_REFCLK_N_IN
  create_bd_pin -dir I -from 0 -to 0 -type clk TX_REFCLK_P_IN
  create_bd_pin -dir I -from 19 -to 0 acr_cts
  create_bd_pin -dir I -from 19 -to 0 acr_n
  create_bd_pin -dir I acr_valid
  create_bd_pin -dir I altclk
  create_bd_pin -dir I -type rst aresetn1
  create_bd_pin -dir O -type intr iic2intc_irpt
  create_bd_pin -dir O -type intr irq
  create_bd_pin -dir O -type intr irq1
  create_bd_pin -dir I -type clk s_axis_aclk
  create_bd_pin -dir I -type clk s_axis_audio_aclk
  create_bd_pin -dir I -type rst s_axis_audio_aresetn
  create_bd_pin -dir I -type rst sc_aresetn
  create_bd_pin -dir O -type clk tx_tmds_clk
  create_bd_pin -dir O -type intr vmix_intr

  # Create instance: hdmi_tx_phy
  create_hier_cell_hdmi_tx_phy $hier_obj hdmi_tx_phy

  # Create instance: smartconnect_vmix_0, and set properties
  set smartconnect_vmix_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect smartconnect_vmix_0 ]
  set_property -dict [ list \
   CONFIG.ADVANCED_PROPERTIES {    __view__ { functional { S07_Buffer { R_SIZE 1024 } S00_Buffer { R_SIZE 1024 } S06_Buffer { R_SIZE 1024 } S05_Buffer { R_SIZE 1024 } S01_Buffer { R_SIZE 1024 } S02_Buffer { R_SIZE 1024 } M00_Buffer { R_SIZE 1024 } S03_Buffer { R_SIZE 1024 } S04_Buffer { R_SIZE 1024 } } }   } \
   CONFIG.NUM_SI {4} \
 ] $smartconnect_vmix_0

  # Create instance: smartconnect_vmix_1, and set properties
  set smartconnect_vmix_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect smartconnect_vmix_1 ]
  set_property -dict [ list \
   CONFIG.ADVANCED_PROPERTIES {    __view__ { functional { S07_Buffer { R_SIZE 1024 } S00_Buffer { R_SIZE 1024 } S06_Buffer { R_SIZE 1024 } S05_Buffer { R_SIZE 1024 } S01_Buffer { R_SIZE 1024 } S02_Buffer { R_SIZE 1024 } M00_Buffer { R_SIZE 1024 } S03_Buffer { R_SIZE 1024 } S04_Buffer { R_SIZE 1024 } } }   } \
   CONFIG.NUM_SI {4} \
 ] $smartconnect_vmix_1

  # Create instance: v_hdmi_tx_ss_0, and set properties
  set v_hdmi_tx_ss_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:v_hdmi_tx_ss v_hdmi_tx_ss_0 ]
  set_property -dict [ list \
   CONFIG.C_INCLUDE_LOW_RESO_VID {true} \
   CONFIG.C_INCLUDE_YUV420_SUP {true} \
   CONFIG.C_INPUT_PIXELS_PER_CLOCK {4} \
   CONFIG.C_MAX_BITS_PER_COMPONENT {8} \
 ] $v_hdmi_tx_ss_0

  # Create instance: v_mix_0, and set properties
  set v_mix_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:v_mix v_mix_0 ]
  set_property -dict [ list \
   CONFIG.AXIMM_ADDR_WIDTH {64} \
   CONFIG.AXIMM_BURST_LENGTH {256} \
   CONFIG.AXIMM_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO10_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO11_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO12_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO13_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO14_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO15_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO16_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO1_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO2_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO3_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO4_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO5_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO6_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO7_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO8_DATA_WIDTH {256} \
   CONFIG.C_M_AXI_MM_VIDEO9_DATA_WIDTH {256} \
   CONFIG.LAYER10_ALPHA {true} \
   CONFIG.LAYER10_VIDEO_FORMAT {10} \
   CONFIG.LAYER11_ALPHA {true} \
   CONFIG.LAYER11_VIDEO_FORMAT {10} \
   CONFIG.LAYER12_ALPHA {true} \
   CONFIG.LAYER12_VIDEO_FORMAT {10} \
   CONFIG.LAYER13_ALPHA {true} \
   CONFIG.LAYER13_VIDEO_FORMAT {26} \
   CONFIG.LAYER1_ALPHA {true} \
   CONFIG.LAYER1_VIDEO_FORMAT {29} \
   CONFIG.LAYER2_ALPHA {true} \
   CONFIG.LAYER2_VIDEO_FORMAT {29} \
   CONFIG.LAYER3_ALPHA {true} \
   CONFIG.LAYER3_VIDEO_FORMAT {29} \
   CONFIG.LAYER4_ALPHA {true} \
   CONFIG.LAYER4_VIDEO_FORMAT {29} \
   CONFIG.LAYER5_ALPHA {true} \
   CONFIG.LAYER5_VIDEO_FORMAT {12} \
   CONFIG.LAYER6_ALPHA {true} \
   CONFIG.LAYER6_VIDEO_FORMAT {12} \
   CONFIG.LAYER7_ALPHA {true} \
   CONFIG.LAYER7_VIDEO_FORMAT {12} \
   CONFIG.LAYER8_ALPHA {true} \
   CONFIG.LAYER8_VIDEO_FORMAT {12} \
   CONFIG.LAYER9_ALPHA {true} \
   CONFIG.LAYER9_VIDEO_FORMAT {26} \
   CONFIG.MAX_DATA_WIDTH {8} \
   CONFIG.NR_LAYERS {10} \
   CONFIG.SAMPLES_PER_CLOCK {4} \
 ] $v_mix_0

  # Create instance: xlconstant_0, and set properties
  set xlconstant_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant xlconstant_0 ]
  set_property -dict [ list \
   CONFIG.CONST_VAL {0} \
   CONFIG.CONST_WIDTH {96} \
 ] $xlconstant_0

  # Create instance: xlslice_20, and set properties
  set xlslice_20 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice xlslice_20 ]
  set_property -dict [ list \
   CONFIG.DIN_FROM {0} \
   CONFIG.DIN_TO {0} \
   CONFIG.DOUT_WIDTH {1} \
 ] $xlslice_20

  # Create interface connections
  connect_bd_intf_net -intf_net Conn1 [get_bd_intf_pins AUDIO_IN] [get_bd_intf_pins v_hdmi_tx_ss_0/AUDIO_IN]
  connect_bd_intf_net -intf_net Conn3 [get_bd_intf_pins HDMI_CTL_IIC] [get_bd_intf_pins hdmi_tx_phy/HDMI_CTL_IIC]
  connect_bd_intf_net -intf_net Conn6 [get_bd_intf_pins S_AXI] [get_bd_intf_pins hdmi_tx_phy/S_AXI]
  connect_bd_intf_net -intf_net Conn8 [get_bd_intf_pins axi4lite] [get_bd_intf_pins hdmi_tx_phy/axi4lite]
  connect_bd_intf_net -intf_net S_AXI_CPU_IN_1 [get_bd_intf_pins S_AXI_CPU_IN] [get_bd_intf_pins v_hdmi_tx_ss_0/S_AXI_CPU_IN]
  connect_bd_intf_net -intf_net hdmi_tx_pipe_M_AXIS [get_bd_intf_pins hdmi_tx_phy/M_AXIS] [get_bd_intf_pins v_hdmi_tx_ss_0/VIDEO_IN]
  connect_bd_intf_net -intf_net hdmi_tx_pipe_status_sb_tx [get_bd_intf_pins hdmi_tx_phy/status_sb_tx] [get_bd_intf_pins v_hdmi_tx_ss_0/SB_STATUS_IN]
  connect_bd_intf_net -intf_net s_axi_lite_vmix_1 [get_bd_intf_pins s_axi_ctrl_vmix] [get_bd_intf_pins v_mix_0/s_axi_CTRL]
  connect_bd_intf_net -intf_net smartconnect_0_M00_AXI [get_bd_intf_pins vmix_mm_axi_vid_rd_0] [get_bd_intf_pins smartconnect_vmix_0/M00_AXI]
  connect_bd_intf_net -intf_net smartconnect_1_M00_AXI [get_bd_intf_pins vmix_mm_axi_vid_rd_1] [get_bd_intf_pins smartconnect_vmix_1/M00_AXI]
  connect_bd_intf_net -intf_net v_hdmi_tx_ss_0_DDC_OUT [get_bd_intf_pins TX_DDC_OUT] [get_bd_intf_pins v_hdmi_tx_ss_0/DDC_OUT]
  connect_bd_intf_net -intf_net v_hdmi_tx_ss_0_LINK_DATA0_OUT [get_bd_intf_pins hdmi_tx_phy/tx_axi4s_ch0] [get_bd_intf_pins v_hdmi_tx_ss_0/LINK_DATA0_OUT]
  connect_bd_intf_net -intf_net v_hdmi_tx_ss_0_LINK_DATA1_OUT [get_bd_intf_pins hdmi_tx_phy/tx_axi4s_ch1] [get_bd_intf_pins v_hdmi_tx_ss_0/LINK_DATA1_OUT]
  connect_bd_intf_net -intf_net v_hdmi_tx_ss_0_LINK_DATA2_OUT [get_bd_intf_pins hdmi_tx_phy/tx_axi4s_ch2] [get_bd_intf_pins v_hdmi_tx_ss_0/LINK_DATA2_OUT]
  connect_bd_intf_net -intf_net v_mix_0_m_axi_mm_video1 [get_bd_intf_pins smartconnect_vmix_0/S00_AXI] [get_bd_intf_pins v_mix_0/m_axi_mm_video1]
  connect_bd_intf_net -intf_net v_mix_0_m_axi_mm_video2 [get_bd_intf_pins smartconnect_vmix_0/S01_AXI] [get_bd_intf_pins v_mix_0/m_axi_mm_video2]
  connect_bd_intf_net -intf_net v_mix_0_m_axi_mm_video3 [get_bd_intf_pins smartconnect_vmix_1/S00_AXI] [get_bd_intf_pins v_mix_0/m_axi_mm_video3]
  connect_bd_intf_net -intf_net v_mix_0_m_axi_mm_video4 [get_bd_intf_pins smartconnect_vmix_1/S01_AXI] [get_bd_intf_pins v_mix_0/m_axi_mm_video4]
  connect_bd_intf_net -intf_net v_mix_0_m_axi_mm_video5 [get_bd_intf_pins smartconnect_vmix_0/S02_AXI] [get_bd_intf_pins v_mix_0/m_axi_mm_video5]
  connect_bd_intf_net -intf_net v_mix_0_m_axi_mm_video6 [get_bd_intf_pins smartconnect_vmix_0/S03_AXI] [get_bd_intf_pins v_mix_0/m_axi_mm_video6]
  connect_bd_intf_net -intf_net v_mix_0_m_axi_mm_video7 [get_bd_intf_pins smartconnect_vmix_1/S02_AXI] [get_bd_intf_pins v_mix_0/m_axi_mm_video7]
  connect_bd_intf_net -intf_net v_mix_0_m_axi_mm_video8 [get_bd_intf_pins smartconnect_vmix_1/S03_AXI] [get_bd_intf_pins v_mix_0/m_axi_mm_video8]
  connect_bd_intf_net -intf_net v_mix_0_m_axi_mm_video9 [get_bd_intf_pins vmix_mm_axi_vid_rd_2] [get_bd_intf_pins v_mix_0/m_axi_mm_video9]
  connect_bd_intf_net -intf_net v_mix_0_m_axis_video [get_bd_intf_pins hdmi_tx_phy/s_axis_video] [get_bd_intf_pins v_mix_0/m_axis_video]

  # Create port connections
  connect_bd_net -net ARESETN_2 [get_bd_pins ARESETN] [get_bd_pins hdmi_tx_phy/ARESETN] -boundary_type upper
  connect_bd_net -net ARESETN_2 [get_bd_pins ARESETN] [get_bd_pins v_hdmi_tx_ss_0/s_axi_cpu_aresetn] -boundary_type upper
  connect_bd_net -net IDT_8T49N241_LOL_IN_1 [get_bd_pins IDT_8T49N241_LOL_IN] [get_bd_pins hdmi_tx_phy/IDT_8T49N241_LOL_IN]
  connect_bd_net -net RX_DATA_IN_rxn_1 [get_bd_pins RX_DATA_IN_rxn] [get_bd_pins hdmi_tx_phy/RX_DATA_IN_rxn]
  connect_bd_net -net RX_DATA_IN_rxp_1 [get_bd_pins RX_DATA_IN_rxp] [get_bd_pins hdmi_tx_phy/RX_DATA_IN_rxp]
  connect_bd_net -net TX_HPD_IN_1 [get_bd_pins TX_HPD_IN] [get_bd_pins v_hdmi_tx_ss_0/hpd]
  connect_bd_net -net TX_REFCLK_N_IN_1 [get_bd_pins TX_REFCLK_N_IN] [get_bd_pins hdmi_tx_phy/TX_REFCLK_N_IN]
  connect_bd_net -net TX_REFCLK_P_IN_1 [get_bd_pins TX_REFCLK_P_IN] [get_bd_pins hdmi_tx_phy/TX_REFCLK_P_IN]
  connect_bd_net -net acr_cts_1 [get_bd_pins acr_cts] [get_bd_pins v_hdmi_tx_ss_0/acr_cts]
  connect_bd_net -net acr_n_1 [get_bd_pins acr_n] [get_bd_pins v_hdmi_tx_ss_0/acr_n]
  connect_bd_net -net acr_valid_1 [get_bd_pins acr_valid] [get_bd_pins v_hdmi_tx_ss_0/acr_valid]
  connect_bd_net -net altclk_1 [get_bd_pins altclk] [get_bd_pins hdmi_tx_phy/altclk] -boundary_type upper
  connect_bd_net -net altclk_1 [get_bd_pins altclk] [get_bd_pins v_hdmi_tx_ss_0/s_axi_cpu_aclk] -boundary_type upper
  connect_bd_net -net aresetn1_1 [get_bd_pins aresetn1] [get_bd_pins hdmi_tx_phy/aresetn1] -boundary_type upper
  connect_bd_net -net aresetn1_1 [get_bd_pins aresetn1] [get_bd_pins v_hdmi_tx_ss_0/s_axis_video_aresetn] -boundary_type upper
  connect_bd_net -net aresetn_1 [get_bd_pins sc_aresetn] [get_bd_pins smartconnect_vmix_0/aresetn] -boundary_type upper
  connect_bd_net -net aresetn_1 [get_bd_pins sc_aresetn] [get_bd_pins smartconnect_vmix_1/aresetn] -boundary_type upper
  connect_bd_net -net clk_wizard_0_clk_out3 [get_bd_pins s_axis_aclk] [get_bd_pins hdmi_tx_phy/aclk] -boundary_type upper
  connect_bd_net -net clk_wizard_0_clk_out3 [get_bd_pins s_axis_aclk] [get_bd_pins smartconnect_vmix_0/aclk] -boundary_type upper
  connect_bd_net -net clk_wizard_0_clk_out3 [get_bd_pins s_axis_aclk] [get_bd_pins smartconnect_vmix_1/aclk] -boundary_type upper
  connect_bd_net -net clk_wizard_0_clk_out3 [get_bd_pins s_axis_aclk] [get_bd_pins v_hdmi_tx_ss_0/s_axis_video_aclk] -boundary_type upper
  connect_bd_net -net clk_wizard_0_clk_out3 [get_bd_pins s_axis_aclk] [get_bd_pins v_mix_0/ap_clk] -boundary_type upper
  connect_bd_net -net hdmi_tx_phy_tx_tmds_clk [get_bd_pins tx_tmds_clk] [get_bd_pins hdmi_tx_phy/tx_tmds_clk]
  connect_bd_net -net hdmi_tx_pipe_TX_EN_OUT [get_bd_pins TX_EN_OUT] [get_bd_pins hdmi_tx_phy/TX_EN_OUT]
  connect_bd_net -net hdmi_tx_pipe_iic2intc_irpt [get_bd_pins iic2intc_irpt] [get_bd_pins hdmi_tx_phy/iic2intc_irpt]
  connect_bd_net -net hdmi_tx_pipe_irq [get_bd_pins irq] [get_bd_pins hdmi_tx_phy/irq]
  connect_bd_net -net hdmi_tx_pipe_tx_usrclk [get_bd_pins hdmi_tx_phy/tx_usrclk] [get_bd_pins v_hdmi_tx_ss_0/link_clk]
  connect_bd_net -net hdmi_tx_pipe_tx_video_clk [get_bd_pins hdmi_tx_phy/tx_video_clk] [get_bd_pins v_hdmi_tx_ss_0/video_clk]
  connect_bd_net -net hdmi_tx_pipe_txn_0 [get_bd_pins TX_DATA_OUT_txn] [get_bd_pins hdmi_tx_phy/TX_DATA_OUT_txn]
  connect_bd_net -net hdmi_tx_pipe_txp_0 [get_bd_pins TX_DATA_OUT_txp] [get_bd_pins hdmi_tx_phy/TX_DATA_OUT_txp]
  connect_bd_net -net ps_gpio_1 [get_bd_pins Din] [get_bd_pins xlslice_20/Din]
  connect_bd_net -net s_axis_audio_aclk_1 [get_bd_pins s_axis_audio_aclk] [get_bd_pins v_hdmi_tx_ss_0/s_axis_audio_aclk]
  connect_bd_net -net s_axis_audio_aresetn_1 [get_bd_pins s_axis_audio_aresetn] [get_bd_pins v_hdmi_tx_ss_0/s_axis_audio_aresetn]
  connect_bd_net -net v_hdmi_tx_ss_0_irq [get_bd_pins irq1] [get_bd_pins v_hdmi_tx_ss_0/irq]
  connect_bd_net -net v_mix_0_interrupt [get_bd_pins vmix_intr] [get_bd_pins v_mix_0/interrupt]
  connect_bd_net -net xlconstant_0_dout [get_bd_pins v_mix_0/s_axis_video_TDATA] [get_bd_pins v_mix_0/s_axis_video_TVALID] -boundary_type upper
  connect_bd_net -net xlconstant_0_dout [get_bd_pins v_mix_0/s_axis_video_TDATA] [get_bd_pins xlconstant_0/dout] -boundary_type upper
  connect_bd_net -net xlslice_20_Dout [get_bd_pins v_mix_0/ap_rst_n] [get_bd_pins xlslice_20/Dout]

  # Restore current instance
  current_bd_instance $oldCurInst
}

# Hierarchical cell: audio_pipe
proc create_hier_cell_audio_pipe { parentCell nameHier } {

  variable script_folder

  if { $parentCell eq "" || $nameHier eq "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2092 -severity "ERROR" "create_hier_cell_audio_pipe() - Empty argument(s)!"}
     return
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2090 -severity "ERROR" "Unable to find parent cell <$parentCell>!"}
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2091 -severity "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj

  # Create cell and set as current instance
  set hier_obj [create_bd_cell -type hier $nameHier]
  current_bd_instance $hier_obj

  # Create interface pins
  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 CLK_IN_AUDIO

  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 m_axi_mm2s

  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:axis_rtl:1.0 m_axis_mm2s

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 s_axi_ctrl_acr

  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 s_axi_ctrl_aud_for


  # Create pins
  create_bd_pin -dir O -from 19 -to 0 aud_acr_cts_out
  create_bd_pin -dir O -from 19 -to 0 aud_acr_n_out
  create_bd_pin -dir O aud_acr_valid_out
  create_bd_pin -dir O -from 0 -to 0 -type clk aud_clk
  create_bd_pin -dir O -type rst aud_resetn_out
  create_bd_pin -dir I -type rst ext_reset_in
  create_bd_pin -dir I -type clk hdmi_clk
  create_bd_pin -dir O -type intr irq_mm2s
  create_bd_pin -dir I -type clk s_axi_lite_aclk
  create_bd_pin -dir I -type rst s_axi_lite_aresetn

  # Create instance: audio_formatter_0, and set properties
  set audio_formatter_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:audio_formatter audio_formatter_0 ]
  set_property -dict [ list \
   CONFIG.C_INCLUDE_S2MM {0} \
 ] $audio_formatter_0

  # Create instance: hdmi_acr_ctrl_0, and set properties
  set hdmi_acr_ctrl_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:hdmi_acr_ctrl:1.0 hdmi_acr_ctrl_0 ]
  set_property -dict [ list \
   CONFIG.C_HDMI_VERSION {0} \
] $hdmi_acr_ctrl_0

  # Create instance: proc_sys_reset_aud_clk, and set properties
  set proc_sys_reset_aud_clk [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset proc_sys_reset_aud_clk ]

  # Create instance: refclk_aud
  create_hier_cell_refclk_aud $hier_obj refclk_aud

  # Create instance: xlconstant_0, and set properties
  set xlconstant_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant xlconstant_0 ]
  set_property -dict [ list \
   CONFIG.CONST_VAL {0} \
   CONFIG.CONST_WIDTH {20} \
 ] $xlconstant_0

  # Create instance: xlconstant_1, and set properties
  set xlconstant_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant xlconstant_1 ]

  # Create interface connections
  connect_bd_intf_net -intf_net Conn1 [get_bd_intf_pins m_axi_mm2s] [get_bd_intf_pins audio_formatter_0/m_axi_mm2s]
  connect_bd_intf_net -intf_net Conn2 [get_bd_intf_pins s_axi_ctrl_acr] [get_bd_intf_pins hdmi_acr_ctrl_0/axi]
  connect_bd_intf_net -intf_net Conn3 [get_bd_intf_pins CLK_IN_AUDIO] [get_bd_intf_pins refclk_aud/CLK_IN_D_0]
  connect_bd_intf_net -intf_net audio_formatter_0_m_axis_mm2s [get_bd_intf_pins m_axis_mm2s] [get_bd_intf_pins audio_formatter_0/m_axis_mm2s]
  connect_bd_intf_net -intf_net smartconnect_gp0_M08_AXI [get_bd_intf_pins s_axi_ctrl_aud_for] [get_bd_intf_pins audio_formatter_0/s_axi_lite]

  # Create port connections
  connect_bd_net -net audio_formatter_0_irq_mm2s [get_bd_pins irq_mm2s] [get_bd_pins audio_formatter_0/irq_mm2s]
  connect_bd_net -net clk_wiz_clk_out2 [get_bd_pins s_axi_lite_aclk] [get_bd_pins audio_formatter_0/s_axi_lite_aclk] -boundary_type upper
  connect_bd_net -net clk_wiz_clk_out2 [get_bd_pins s_axi_lite_aclk] [get_bd_pins hdmi_acr_ctrl_0/axi_aclk] -boundary_type upper
  connect_bd_net -net ext_reset_in1_1 [get_bd_pins ext_reset_in] [get_bd_pins proc_sys_reset_aud_clk/ext_reset_in]
  connect_bd_net -net hdmi_acr_ctrl_0_aud_acr_cts_out [get_bd_pins aud_acr_cts_out] [get_bd_pins hdmi_acr_ctrl_0/aud_acr_cts_out]
  connect_bd_net -net hdmi_acr_ctrl_0_aud_acr_n_out [get_bd_pins aud_acr_n_out] [get_bd_pins hdmi_acr_ctrl_0/aud_acr_n_out]
  connect_bd_net -net hdmi_acr_ctrl_0_aud_acr_valid_out [get_bd_pins aud_acr_valid_out] [get_bd_pins hdmi_acr_ctrl_0/aud_acr_valid_out]
  connect_bd_net -net hdmi_acr_ctrl_0_aud_resetn_out [get_bd_pins aud_resetn_out] [get_bd_pins hdmi_acr_ctrl_0/aud_resetn_out]
  connect_bd_net -net hdmi_clk_1 [get_bd_pins hdmi_clk] [get_bd_pins hdmi_acr_ctrl_0/hdmi_clk]
  connect_bd_net -net proc_sys_reset_aud_clk_peripheral_aresetn [get_bd_pins audio_formatter_0/m_axis_mm2s_aresetn] [get_bd_pins proc_sys_reset_aud_clk/peripheral_aresetn]
  connect_bd_net -net proc_sys_reset_aud_clk_peripheral_reset [get_bd_pins audio_formatter_0/aud_mreset] [get_bd_pins proc_sys_reset_aud_clk/peripheral_reset]
  connect_bd_net -net refclk_aud_BUFG_O [get_bd_pins aud_clk] [get_bd_pins audio_formatter_0/aud_mclk] -boundary_type upper
  connect_bd_net -net refclk_aud_BUFG_O [get_bd_pins aud_clk] [get_bd_pins audio_formatter_0/m_axis_mm2s_aclk] -boundary_type upper
  connect_bd_net -net refclk_aud_BUFG_O [get_bd_pins aud_clk] [get_bd_pins hdmi_acr_ctrl_0/aud_clk] -boundary_type upper
  connect_bd_net -net refclk_aud_BUFG_O [get_bd_pins aud_clk] [get_bd_pins proc_sys_reset_aud_clk/slowest_sync_clk] -boundary_type upper
  connect_bd_net -net refclk_aud_BUFG_O [get_bd_pins aud_clk] [get_bd_pins refclk_aud/BUFG_O] -boundary_type upper
  connect_bd_net -net rst_processor_1_100M_peripheral_aresetn [get_bd_pins s_axi_lite_aresetn] [get_bd_pins audio_formatter_0/s_axi_lite_aresetn] -boundary_type upper
  connect_bd_net -net rst_processor_1_100M_peripheral_aresetn [get_bd_pins s_axi_lite_aresetn] [get_bd_pins hdmi_acr_ctrl_0/axi_aresetn] -boundary_type upper
  connect_bd_net -net xlconstant_0_dout [get_bd_pins hdmi_acr_ctrl_0/pll_lock_in] [get_bd_pins xlconstant_1/dout]
  connect_bd_net -net xlconstant_0_dout1 [get_bd_pins hdmi_acr_ctrl_0/aud_acr_cts_in] [get_bd_pins hdmi_acr_ctrl_0/aud_acr_n_in] -boundary_type upper
  connect_bd_net -net xlconstant_0_dout1 [get_bd_pins hdmi_acr_ctrl_0/aud_acr_cts_in] [get_bd_pins hdmi_acr_ctrl_0/aud_acr_valid_in] -boundary_type upper
  connect_bd_net -net xlconstant_0_dout1 [get_bd_pins hdmi_acr_ctrl_0/aud_acr_cts_in] [get_bd_pins xlconstant_0/dout] -boundary_type upper

  # Restore current instance
  current_bd_instance $oldCurInst
}

# Hierarchical cell: refclk_aud
proc create_hier_cell_refclk_aud { parentCell nameHier } {

  variable script_folder

  if { $parentCell eq "" || $nameHier eq "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2092 -severity "ERROR" "create_hier_cell_refclk_aud() - Empty argument(s)!"}
     return
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2090 -severity "ERROR" "Unable to find parent cell <$parentCell>!"}
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2091 -severity "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj

  # Create cell and set as current instance
  set hier_obj [create_bd_cell -type hier $nameHier]
  current_bd_instance $hier_obj

  # Create interface pins
  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 CLK_IN_D_0


  # Create pins
  create_bd_pin -dir O -from 0 -to 0 -type clk BUFG_O

  # Create instance: util_ds_buf_0, and set properties
  set util_ds_buf_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf util_ds_buf_0 ]

  # Create instance: util_ds_buf_1, and set properties
  set util_ds_buf_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf util_ds_buf_1 ]
  set_property -dict [ list \
   CONFIG.C_BUF_TYPE {BUFG} \
 ] $util_ds_buf_1

  # Create interface connections
  connect_bd_intf_net -intf_net Conn3 [get_bd_intf_pins CLK_IN_D_0] [get_bd_intf_pins util_ds_buf_0/CLK_IN_D]

  # Create port connections
  connect_bd_net -net util_ds_buf_0_IBUF_OUT [get_bd_pins util_ds_buf_0/IBUF_OUT] [get_bd_pins util_ds_buf_1/BUFG_I]
  connect_bd_net -net util_ds_buf_1_BUFG_O [get_bd_pins BUFG_O] [get_bd_pins util_ds_buf_1/BUFG_O]

  # Restore current instance
  current_bd_instance $oldCurInst
}

# Create audio pipe and display pipe
create_hier_cell_audio_pipe [current_bd_instance .] audio_pipe
create_hier_cell_display_pipe [current_bd_instance .] display_pipe

# External ports
create_bd_port -dir I IDT_8T49N241_LOL_IN
create_bd_port -dir I -from 3 -to 0 RX_DATA_IN_rxn
create_bd_port -dir I -from 3 -to 0 RX_DATA_IN_rxp
create_bd_port -dir O -from 3 -to 0 TX_DATA_OUT_txn
create_bd_port -dir O -from 3 -to 0 TX_DATA_OUT_txp
create_bd_port -dir O -from 0 -to 0 TX_EN_OUT
create_bd_port -dir I TX_HPD_IN
create_bd_port -dir I TX_REFCLK_N_IN
create_bd_port -dir I TX_REFCLK_P_IN
create_bd_intf_port -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 HDMI_CTL_IIC
create_bd_intf_port -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 TX_DDC_OUT
create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 CLK_IN_AUDIO

# Connect external ports
connect_bd_net [get_bd_ports IDT_8T49N241_LOL_IN] [get_bd_pins display_pipe/IDT_8T49N241_LOL_IN]
connect_bd_net [get_bd_ports RX_DATA_IN_rxn] [get_bd_pins display_pipe/RX_DATA_IN_rxn]
connect_bd_net [get_bd_ports RX_DATA_IN_rxp] [get_bd_pins display_pipe/RX_DATA_IN_rxp]
connect_bd_net [get_bd_ports TX_DATA_OUT_txn] [get_bd_pins display_pipe/TX_DATA_OUT_txn]
connect_bd_net [get_bd_ports TX_DATA_OUT_txp] [get_bd_pins display_pipe/TX_DATA_OUT_txp]
connect_bd_net [get_bd_ports TX_EN_OUT] [get_bd_pins display_pipe/TX_EN_OUT]
connect_bd_net [get_bd_ports TX_HPD_IN] [get_bd_pins display_pipe/TX_HPD_IN]
connect_bd_net [get_bd_ports TX_REFCLK_N_IN] [get_bd_pins display_pipe/TX_REFCLK_N_IN]
connect_bd_net [get_bd_ports TX_REFCLK_P_IN] [get_bd_pins display_pipe/TX_REFCLK_P_IN]
connect_bd_intf_net [get_bd_intf_ports HDMI_CTL_IIC] [get_bd_intf_pins display_pipe/HDMI_CTL_IIC]
connect_bd_intf_net [get_bd_intf_ports TX_DDC_OUT] [get_bd_intf_pins display_pipe/TX_DDC_OUT]
connect_bd_intf_net [get_bd_intf_ports CLK_IN_AUDIO] [get_bd_intf_pins audio_pipe/CLK_IN_AUDIO]

# Connect the AXI lite interfaces to M_AXI_FPD
lappend m_axi_fpd_ports "display_pipe/S_AXI"
lappend m_axi_fpd_ports "display_pipe/S_AXI_CPU_IN"
lappend m_axi_fpd_ports "display_pipe/axi4lite"
lappend m_axi_fpd_ports "display_pipe/s_axi_ctrl_vmix"
lappend m_axi_fpd_ports "audio_pipe/s_axi_ctrl_acr"
lappend m_axi_fpd_ports "audio_pipe/s_axi_ctrl_aud_for"

# Connect clocks
connect_bd_net [get_bd_pins clk_wiz_0/clk_100M] [get_bd_pins display_pipe/altclk]
connect_bd_net [get_bd_pins clk_wiz_0/clk_150M] [get_bd_pins display_pipe/s_axis_aclk]
connect_bd_net [get_bd_pins clk_wiz_0/clk_100M] [get_bd_pins audio_pipe/s_axi_lite_aclk]

# Connect resets
connect_bd_net [get_bd_pins reset_100M/peripheral_aresetn] [get_bd_pins display_pipe/ARESETN]
connect_bd_net [get_bd_pins reset_150M/peripheral_aresetn] [get_bd_pins display_pipe/aresetn1]
connect_bd_net [get_bd_pins reset_150M/interconnect_aresetn] [get_bd_pins display_pipe/sc_aresetn]
connect_bd_net [get_bd_pins versal_cips_0/pl0_resetn] [get_bd_pins audio_pipe/ext_reset_in]
connect_bd_net [get_bd_pins reset_100M/peripheral_aresetn] [get_bd_pins audio_pipe/s_axi_lite_aresetn]

# EMIO GPIP
connect_bd_net [get_bd_pins versal_cips_0/LPD_GPIO_o] [get_bd_pins display_pipe/Din]

# Connections between audio_pipe and display_pipe
connect_bd_intf_net -boundary_type upper [get_bd_intf_pins audio_pipe/m_axis_mm2s] [get_bd_intf_pins display_pipe/AUDIO_IN]
connect_bd_net [get_bd_pins audio_pipe/aud_acr_cts_out] [get_bd_pins display_pipe/acr_cts] -boundary_type upper
connect_bd_net [get_bd_pins audio_pipe/aud_acr_n_out] [get_bd_pins display_pipe/acr_n] -boundary_type upper
connect_bd_net [get_bd_pins audio_pipe/aud_acr_valid_out] [get_bd_pins display_pipe/acr_valid] -boundary_type upper
connect_bd_net [get_bd_pins audio_pipe/aud_clk] [get_bd_pins display_pipe/s_axis_audio_aclk] -boundary_type upper
connect_bd_net [get_bd_pins audio_pipe/aud_resetn_out] [get_bd_pins display_pipe/s_axis_audio_aresetn] -boundary_type upper
connect_bd_net [get_bd_pins display_pipe/tx_tmds_clk] [get_bd_pins audio_pipe/hdmi_clk] -boundary_type upper

# Interrupts
lappend intr_list "display_pipe/iic2intc_irpt"
lappend intr_list "display_pipe/irq"
lappend intr_list "display_pipe/irq1"
lappend intr_list "display_pipe/vmix_intr"
lappend intr_list "audio_pipe/irq_mm2s"

# NOC ports
lappend noc_ports [list "display_pipe/vmix_mm_axi_vid_rd_0" "clk_wiz_0/clk_150M"]
lappend noc_ports [list "display_pipe/vmix_mm_axi_vid_rd_1" "clk_wiz_0/clk_150M"]
lappend noc_ports [list "display_pipe/vmix_mm_axi_vid_rd_2" "clk_wiz_0/clk_150M"]
lappend noc_ports [list "audio_pipe/m_axi_mm2s" "audio_pipe/aud_clk"]

###########################################################
# clk_sel outputs
###########################################################

# Add constant for the CAM1 and CAM3 CLK_SEL pin (10b for all Versal boards)
set clk_sel [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant clk_sel]
set_property -dict [list CONFIG.CONST_WIDTH {2} CONFIG.CONST_VAL {0x02}] $clk_sel
create_bd_port -dir O clk_sel
connect_bd_net [get_bd_ports clk_sel] [get_bd_pins clk_sel/dout]

###########################################################
# Reserved GPIO
###########################################################

# Add and configure GPIO for the reserved GPIOs
set rsvd_gpio [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio rsvd_gpio]
set_property -dict [list CONFIG.C_GPIO_WIDTH {10} CONFIG.C_ALL_OUTPUTS {1}] $rsvd_gpio
connect_bd_net [get_bd_pins clk_wiz_0/clk_100M] [get_bd_pins rsvd_gpio/s_axi_aclk]
connect_bd_net [get_bd_pins reset_100M/peripheral_aresetn] [get_bd_pins rsvd_gpio/s_axi_aresetn]
lappend m_axi_fpd_ports "rsvd_gpio/S_AXI"
create_bd_intf_port -mode Master -vlnv xilinx.com:interface:gpio_rtl:1.0 rsvd_gpio
connect_bd_intf_net [get_bd_intf_pins rsvd_gpio/GPIO] [get_bd_intf_ports rsvd_gpio]

###########################################################
# AXI Lite ports
###########################################################

# Connect AXI lite interfaces to M_AXI_FPD
set num_ports [llength $m_axi_fpd_ports]
set_property -dict [list CONFIG.NUM_MI $num_ports] [get_bd_cells axi_smc_fpd]
set port_index 0
foreach port $m_axi_fpd_ports {
  set padded_port_index [format "%02d" $port_index]
  connect_bd_intf_net [get_bd_intf_pins axi_smc_fpd/M${padded_port_index}_AXI] -boundary_type upper [get_bd_intf_pins $port]
  set port_index [expr {$port_index+1}]
}

###########################################################
# NOC ports
###########################################################

# Connect clocks first
set num_noc_clks [get_property CONFIG.NUM_CLKS [get_bd_cells axi_noc_0]]
set clk_index $num_noc_clks

set clock_dict {}
set clock_list []
foreach port $noc_ports {
    set clock_name [lindex $port 1]
    puts "clock_name is $clock_name"
    # Check if this clock is already in the dictionary
    if {![dict exists $clock_dict $clock_name]} {
        puts "adding $clock_name"
        # If not, add it to the dictionary with the current index as its value
        dict set clock_dict $clock_name $clk_index
        lappend clock_list $clock_name
        # Increment the index for the next unique clock name
        incr clk_index
    } else {
      puts "nothing"
    }
}
# Increase the number of clock inputs by length of clock_list
set noc_clks_to_add [llength $clock_list]
set num_noc_clks [expr {$num_noc_clks+$noc_clks_to_add}]
set_property -dict [list CONFIG.NUM_CLKS $num_noc_clks] [get_bd_cells axi_noc_0]
foreach clk $clock_list {
  set clk_index [dict get $clock_dict $clk]
  connect_bd_net [get_bd_pins $clk] [get_bd_pins axi_noc_0/aclk${clk_index}]
}

# Connect AXI MM interfaces to the NOC
# Increase the number of AXI slave ports
set noc_ports_to_add [llength $noc_ports]
set num_noc_ports [get_property CONFIG.NUM_SI [get_bd_cells axi_noc_0]]
set port_index $num_noc_ports
set num_noc_ports [expr {$num_noc_ports+$noc_ports_to_add}]
set last_port_index [expr {$num_noc_ports-1}]
set_property -dict [list CONFIG.NUM_SI $num_noc_ports] [get_bd_cells axi_noc_0]
set index 0
foreach port $noc_ports {
  set port_name [lindex $port 0]
  set clock_name [lindex $port 1]
  set clock_index [dict get $clock_dict $clock_name]
  set padded_port_index [format "%02d" $port_index]
  set_property -dict [list \
    CONFIG.CONNECTIONS [list MC_${index} { read_bw {1720} write_bw {1720} read_avg_burst {4} write_avg_burst {4}} ] \
  ] [get_bd_intf_pins /axi_noc_0/S${padded_port_index}_AXI]
  connect_bd_intf_net [get_bd_intf_pins $port_name] -boundary_type upper [get_bd_intf_pins axi_noc_0/S${padded_port_index}_AXI]
  # Append the slave port name to the associated busif parameter
  set busif [get_property CONFIG.ASSOCIATED_BUSIF [get_bd_pins /axi_noc_0/aclk$clock_index]]
  if { [llength $busif] != 0 } {
    append busif ":"
  }
  append busif "S${padded_port_index}_AXI"
  set_property -dict [list CONFIG.ASSOCIATED_BUSIF $busif] [get_bd_pins /axi_noc_0/aclk$clock_index]
  set port_index [expr {$port_index+1}]
  if { $index == 3 } {
    set index 0
  } else {
    set index [expr {$index+1}]
  }
}

# Connect the interrupts
if { 0 } {
  # Use this if we need to enable interrupts in the PS
  set num_intr [llength $intr_list]
  set ps_irq_usage_list {}
  for {set i 0} {$i < 16} {incr i} {
    set flag [expr {$i < $num_intr ? 1 : 0}]
    lappend ps_irq_usage_list [list "CH$i" $flag]
  }
  set_property -dict [list \
    CONFIG.PS_PMC_CONFIG [ list PS_IRQ_USAGE $ps_irq_usage_list ]  \
  ] [get_bd_cells versal_cips_0]
}
set intr_index 0
foreach intr $intr_list {
  connect_bd_net [get_bd_pins $intr] [get_bd_pins versal_cips_0/pl_ps_irq$intr_index]
  set intr_index [expr {$intr_index+1}]
}

######################################################################################
# Platform setup
######################################################################################

# Enable all unused slave ports of axi_noc_0 for accelerators
set n_ports [get_property CONFIG.NUM_SI [get_bd_cells /axi_noc_0]]
set param_list {}
for {set i $n_ports} {$i <= 21} {incr i} {
  set paddedIndex [format "%02d" $i]
  lappend param_list "S${paddedIndex}_AXI"
  lappend param_list [list memport "MIG" sptag "NOC_S${paddedIndex}" memory "" is_range "true"]
}
set_property PFM.AXI_PORT $param_list [get_bd_cells /axi_noc_0]

# Enable all unused master ports of axi_smc_lpd for accelerators
set n_ports [get_property CONFIG.NUM_MI [get_bd_cells /axi_smc_lpd]]
set param_list {}
for {set i $n_ports} {$i <= 15} {incr i} {
  set paddedIndex [format "%02d" $i]
  lappend param_list "M${paddedIndex}_AXI"
  lappend param_list {memport "M_AXI_GP" sptag "" memory "" is_range "true"}
}
set_property PFM.AXI_PORT $param_list [get_bd_cells /axi_smc_lpd]

# Enable platform clocks
set_property PFM.CLOCK { \
  clk_100M {id "1" is_default "false" proc_sys_reset "/reset_100M" status "fixed" freq_hz "100000000"} \
  clk_150M {id "2" is_default "true" proc_sys_reset "/reset_150M" status "fixed" freq_hz "150000000"} \
  clk_200M {id "4" is_default "false" proc_sys_reset "/reset_200M" status "fixed" freq_hz "200000000"} \
} [get_bd_cells /clk_wiz_0]

# Enable platform interrupts through AXI Interrupt controller
set_property PFM.IRQ {intr { id 0 range 32 }} [get_bd_cells /axi_intc_0]

set_property pfm_name "xilinx:$board_name:$target:1.0" [get_files -all "$block_name.bd"]

# Assign addresses
assign_bd_address

validate_bd_design
save_bd_design
