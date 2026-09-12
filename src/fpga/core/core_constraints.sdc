# Game.com clocks and external-memory budgets. APF creates the 74.25 MHz
# clocks and invokes derive_pll_clocks before reading this file.
set mem_clock {ic|machine_pll|pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}
set dram_clock {ic|machine_pll|pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}
set sys_clock {ic|machine_pll|pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk}
set vid_clock {ic|machine_pll|pll_i|general[3].gpll~PLL_OUTPUT_COUNTER|divclk}
set vid90_clock {ic|machine_pll|pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk}
create_generated_clock -name sdram_clk -source [get_pins $dram_clock] [get_ports dram_clk]
# Max/min delays between asynchronous groups preserve Cyclone V skew checks.
# Related clocks retain synchronous timing; per-node bounds below override these.
set async_groups [list \
    [list bridge_spiclk] [list clk_74a] [list clk_74b] \
    [list $mem_clock $dram_clock $sys_clock $vid_clock $vid90_clock sdram_clk] \
    [list {ic|audio_pll|mf_audio_pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk} {ic|audio_pll|mf_audio_pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}]]
foreach launch_group $async_groups {
    foreach capture_group $async_groups {
        if {$launch_group eq $capture_group} { continue }
        set_max_delay 100.0 -from [get_clocks $launch_group] -to [get_clocks $capture_group]
        set_min_delay -100.0 -from [get_clocks $launch_group] -to [get_clocks $capture_group]
    }
}
derive_clock_uncertainty

# Node-specific maxima take precedence over the clock-pair defaults.
foreach {launch capture budget direct gray} {
    {*loader|transfer_fifo|wr_gray[*]} {*loader|transfer_fifo|wr_gray_rd1[*]} 10.0 1 1
    {*loader|transfer_fifo|rd_gray[*]} {*loader|transfer_fifo|rd_gray_wr1[*]} 10.0 1 1
    {*loader|committed_gray[*]} {*loader|committed_gray_b1[*]} 10.0 1 1
    {*loader|announced_bytes[*]} {*loader|mem_size[*]} 20.0 1 0
    {*loader|active_bios} {*loader|mem_bios} 20.0 0 0
    {*loader|backing_base[*]} {*loader|mem_base[*]} 20.0 1 0
    {*loader|input_crc[*]} {*loader|expected_crc_m1[*]} 20.0 1 0
    {*loader|mem_result_crc[*]} {*loader|readback_crc32[*]} 20.0 1 0
    {*loader|rom_size[*]} {*ic|rom_size_s[*]} 20.0 1 0
    {*loader|mem_crc_bad} {*loader|bios_loaded} 20.0 0 0
    {*loader|mem_crc_bad} {*loader|cart_loaded} 20.0 0 0
    {*loader|mem_crc_bad} {*loader|error[*]} 20.0 0 0
    {*rtc_adapter|mailbox_data[*]} {*rtc_adapter|host_time[*]} 50.0 0 0
    {*audio_transport|held_sample[*]} {*audio_transport|next_sample[*]} 50.0 1 0
} {
    set sources [get_registers $launch]
    set destinations [get_registers $capture]
    set_max_delay $budget -from $sources -to $destinations
    if {$direct} { set_net_delay -max $budget -from $sources -to $destinations }
    if {$gray} { set_max_skew $budget -from $sources -to $destinations }
}

# AS4C32M16MSA-6BIN CL2: tIS/tDS2ns, tIH/tDH1ns, tAC6ns, tOH2.5ns.
# Add 0.4ns board margin. Input capture is on falling clk_mem edges; preserve
# the relationship to the phase-shifted, DDR-forwarded SDRAM clock.
set_output_delay -clock sdram_clk -max 2.4 [get_ports {dram_a[*] dram_ba[*] dram_dq[*] dram_dqm[*] dram_ras_n dram_cas_n dram_we_n dram_cke}]
set_output_delay -clock sdram_clk -min -1.4 [get_ports {dram_a[*] dram_ba[*] dram_dq[*] dram_dqm[*] dram_ras_n dram_cas_n dram_we_n dram_cke}]
set_input_delay -clock sdram_clk -max 6.4 [get_ports {dram_dq[*]}]
set_input_delay -clock sdram_clk -min 2.1 [get_ports {dram_dq[*]}]

# AS6C2016-55 asynchronous SRAM: access55ns is enforced by the loader's
# multi-cycle state machine and the CPU's phi0 bus phases (200ns). Bound
# each FPGA leg explicitly; these constraints don't pretend the RAM is50MHz.
set_max_delay 8.0 -from [get_ports {sram_dq[*]}] -to [get_clocks $mem_clock]
# The CPU return includes its external-bus and DMA data muxes. A 12ns return
# budget plus 14ns outbound + 55ns SRAM access totals81ns, within the CPU's
# ~200ns setup/sample beat. The faster loader keeps its 8ns return budget.
set_max_delay 12.0 -from [get_ports {sram_dq[*]}] -to [get_clocks $sys_clock]
set_min_delay 0.0 -from [get_ports {sram_dq[*]}] -to [get_clocks [list $mem_clock $sys_clock]]
# At60MHz the loader samples at7T: outbound14 + SRAM55 + return8 =77ns
# <116.67ns. With outbound legs bounded0..14ns, its write pulse is at
# least5T-14 =69.33ns (>45ns), address setup is7T-14 =102.67ns (>50ns),
# data setup after the3T drive guard is4T-14 =52.67ns (>25ns), and hold
# is2T-14 =19.33ns (>0ns). Guard3T also exceeds14ns+tOHZ20ns; release2T
# exceeds FPGA14ns before the SRAM's earliest5ns turn-on.
set_max_delay 14.0 -from [get_clocks [list $mem_clock $sys_clock]] -to [get_ports {sram_a[*] sram_dq[*] sram_oe_n sram_we_n sram_ub_n sram_lb_n}]
set_min_delay 0.0 -from [get_clocks [list $mem_clock $sys_clock]] -to [get_ports {sram_a[*] sram_dq[*] sram_oe_n sram_we_n sram_ub_n sram_lb_n}]

# APF board protocols and unused pins retain the framework's electrical
# assignments. SDRAM and SRAM are deliberately excluded from these exceptions.
set_false_path -from [get_ports {bridge_1wire bridge_spimiso bridge_spimosi bridge_spiss port_ir_rx dbg_rx user2 aux_sda vblank cram0_wait cram1_wait cram0_dq[*] cram1_dq[*] cart_tran_* port_tran_*}]
set_false_path -to [get_ports {bridge_1wire bridge_spimiso bridge_spimosi scal_* port_ir_* dbg_tx user1 aux_sda aux_scl vpll_feed cram0_* cram1_* cart_* port_tran_*}]
