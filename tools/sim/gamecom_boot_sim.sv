module gamecom_boot_sim (
    input wire clk_sys, clk_vid, clk_bridge,
    input wire reset_cold, reset_run,
    input wire [31:0] keys,
    input wire [7:0] cart_data,
    input wire rom_ready,
    output wire ram_initialized,
    output wire [20:0] cart_addr,
    output wire cart_rd, slot1, slot2,
    output wire [23:0] video_rgb,
    output wire video_hs, video_vs, video_de, video_skip,
    output wire signed [15:0] audio_sample,
    output wire [15:0] debug_pc,
    output wire [7:0] debug_opcode,
    output wire [5:0] debug_state,
    output wire debug_vram_write,
    output wire [3:0] debug_touch_x, debug_touch_y,
    output wire debug_touch_active,
    output wire [13:0] debug_touch_scan
);
    gamecom_machine dut (
        .clk_sys(clk_sys),.clk_vid(clk_vid),.clk_bridge(clk_bridge),
        .reset_cold(reset_cold),.reset_run(reset_run),.cont1_key(keys),
        .power_pulse(1'b0),.rtc_valid(1'b0),.rtc_date_bcd(32'd0),.rtc_time_bcd(32'd0),
        .cart_data(cart_data),.rom_ready(rom_ready),.ram_initialized(ram_initialized),
        .cart_addr(cart_addr),.cart_rd(cart_rd),.cart_slot1_sel(slot1),.cart_slot2_sel(slot2),
        .video_rgb(video_rgb),.video_hs(video_hs),.video_vs(video_vs),.video_de(video_de),
        .video_skip(video_skip),.audio_sample(audio_sample)
    );
    assign debug_pc = dut.machine.u_cpu.pc_q;
    assign debug_opcode = dut.machine.u_cpu.opcode_q;
    assign debug_state = dut.machine.u_cpu.state_q;
    assign debug_vram_write = dut.machine.vram0_wren_mux_w || dut.machine.vram1_wren_mux_w;
    assign debug_touch_x = dut.touch_x;
    assign debug_touch_y = dut.touch_y;
    assign debug_touch_active = dut.touch_active;
    assign debug_touch_scan = dut.machine.u_input.scan_w;
endmodule
