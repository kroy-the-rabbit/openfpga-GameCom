// Copyright (c) 2026 Jamie Blanks

module GameCom
#(
	// Pocket uses a 30 MHz video clock; MiSTer keeps its original 60 MHz.
	parameter integer VIDEO_CLOCK_DIV = 10,
	parameter STOP_LOGO_FILE = "rtl/gamecom_stop_logo.hex"
)
(
	input wire clk_sys,
	input wire phi0,
	input wire clk_vid,
	input wire phi1,
	input wire reset,
	input wire stop_disable_i,
	input wire warm_boot_i,
	input wire video_reset_i,
	input wire [7:0] cart_din_i,
	input wire rom_read_ready_i,
	input wire uart_rxd_i,
	input wire uart_cts_i,
	input wire uart_dsr_i,
	// buttons_i bit order:
	// [0]=Up [1]=Down [2]=Left [3]=Right [4]=Menu [5]=Sound
	// [6]=A [7]=B [8]=C [9]=D [10]=Pause [11]=Power
	input wire [11:0] buttons_i,
	input wire touch_active_i,
	input wire [3:0] touch_x_i,
	input wire [3:0] touch_y_i,
	input wire video_60hz_i,
	input wire palette_four_color_i,
	input wire cursor_enable_i,
	input wire [3:0] cursor_x_i,
	input wire [3:0] cursor_y_i,
	input wire [7:0] save_din_i,
	input wire [64:0] rtc_i,
	input wire savestate_pause_req_i,
	input wire savestate_mem_active_i,
	input wire [2:0] savestate_mem_type_i,
	input wire [24:0] savestate_mem_addr_i,
	input wire savestate_mem_rd_i,
	input wire savestate_mem_wr_i,
	input wire [7:0] savestate_mem_wdata_i,
	input wire cheat_clear_i,
	input wire [128:0] cheat_code_i,

	output wire ce_pix,
	output wire HBlank,
	output wire HSync,
	output wire VBlank,
	output wire VSync,
	output wire [2:0] shade,

	output wire [20:0] cart_addr_o,
	output wire [7:0] cart_dout_o,
	output wire cart_doe_o,
	output wire cart_rd_o,
	output wire cart_wr_o,
	output wire cart_slot1_sel_o,
	output wire cart_slot2_sel_o,
	output wire [12:0] save_addr_o,
	output wire [7:0] save_dout_o,
	output wire save_rd_o,
	output wire save_wren_o,

	output wire [7:0] cpu_sound_o,
	output wire [15:0] audio_pcm_o,
	output wire cpu_txdb_o,
	output wire cpu_lcd_clk_o,
	output wire cpu_doffb_o,
	output wire uart_rts_o,
	output wire uart_dtr_o,
	output wire savestate_pause_ready_o,
	output wire [7:0] savestate_mem_rdata_o
);

	wire [20:0] cpu_a_w;
	wire [7:0] cpu_d_dout_w;
	wire cpu_d_oe_w;
	wire cpu_mce0b_w;
	wire cpu_mce1b_w;
	wire cpu_ioe0b_w;
	wire cpu_ioe1b_w;
	wire cpu_rdb_w;
	wire cpu_wrb_w;

	wire [12:0] cpu_va_w;
	wire [7:0] cpu_vd_dout_w;
	wire cpu_vd_oe_w;
	wire cpu_vce0b_w;
	wire cpu_vce1b_w;
	wire cpu_vrdb_w;
	wire cpu_vwrb_w;

	wire [7:0] ext_din_w;
	wire [7:0] vram_din_w;
	wire [7:0] io_ram_q_w;
	wire [7:0] vram0_cpu_q_w;
	wire [7:0] vram1_cpu_q_w;
	wire [7:0] vram0_video_q_w;
	wire [7:0] vram1_video_q_w;
	wire [12:0] video_vram_addr_w;
	wire video_native_vblank_w;

	wire [7:0] cpu_sound_w;
	wire [15:0] audio_pcm_w;
	wire cpu_txdb_w;
	wire cpu_clk_w;
	wire cpu_display_page_w;
	wire [1:0] cpu_display_palette_w;
	wire cpu_display_normal_black_w;
	wire cpu_stopped_w;
	wire cpu_doffb_w;
	wire [7:0] cpu_p0_dout_w;
	wire [7:0] cpu_p1_dout_w;
	wire [7:0] cpu_p2_dout_w;
	wire [7:0] cpu_p3_dout_w;
	wire [7:0] cpu_p3_latch_w;
	wire [7:0] cpu_p0_oe_w;
	wire [7:0] cpu_p1_oe_w;
	wire [7:0] cpu_p2_oe_w;
	wire [7:0] cpu_p3_oe_w;
	wire [7:0] cpu_p0_pullup_w;
	wire [7:0] cpu_p1_pullup_w;
	wire [7:0] cpu_p2_pullup_w;
	wire [7:0] cpu_p3_pullup_w;
	wire cpu_pio_scan_update_w;
	wire [7:0] board_p0_din_w;
	wire [7:0] board_p1_din_w;
	wire [7:0] board_p2_din_w;
	wire [7:0] board_p3_din_w;
	wire [7:0] cpu_p3_din_w;
	wire cart_slot1_sel_w;
	wire cart_slot2_sel_w;
	wire cpu_intb_w;
	wire [7:0] cpu_savestate_rdata_w;
	wire cpu_savestate_pause_ready_w;
	wire power_cpu_wake_w;
	wire button_up_i = buttons_i[0];
	wire button_down_i = buttons_i[1];
	wire button_left_i = buttons_i[2];
	wire button_right_i = buttons_i[3];
	wire button_menu_i = buttons_i[4];
	wire button_sound_i = buttons_i[5];
	wire button_a_i = buttons_i[6];
	wire button_b_i = buttons_i[7];
	wire button_c_i = buttons_i[8];
	wire button_d_i = buttons_i[9];
	wire button_pause_i = buttons_i[10];
	wire button_power_i = buttons_i[11];
	reg power_int_sync0_q;
	reg power_int_sync1_q;
	reg power_stop_wake_q;
	reg power_boot_pending_q;
	reg power_boot_armed_q;

	localparam [12:0] WARM_BOOT_TUPLE_BASE = 13'h1EE2;
	localparam [2:0] WARM_BOOT_TUPLE_LAST_IDX = 3'd5;

	reg [2:0] warm_boot_seed_idx_q;
	reg warm_boot_seed_done_q;
	reg [7:0] io_ram_addr_q;
	reg [12:0] cpu_vram_addr_q;

	wire rom_cycle_w = !cpu_mce0b_w;
	wire sram_cycle_w = !cpu_mce1b_w;
	wire io_cycle_w = !cpu_ioe0b_w || !cpu_ioe1b_w;
	// Honor the SM8521's external chip selects directly at the board boundary:
	// MCE0B is the ROM/flash class bus, while MCE1B is the SRAM class bus.
	// The Game.com then resolves BIOS-vs-cart behind the ROM-class bus and
	// maps the SRAM-class bus to the console's external RAM / NVRAM window.
	wire work_ram_sel_w = sram_cycle_w;
	wire cart_sel_w = rom_cycle_w;
	wire vram0_sel_w = !cpu_vce0b_w && cpu_vce1b_w;
	wire vram1_sel_w = !cpu_vce1b_w && cpu_vce0b_w;
	wire vram_sel_conflict_w = !cpu_vce0b_w && !cpu_vce1b_w;

	wire work_ram_wren_w = work_ram_sel_w && !cpu_wrb_w && cpu_d_oe_w;
	wire io_ram_wren_w = io_cycle_w && !cpu_wrb_w && cpu_d_oe_w;
	wire vram0_wren_w = vram0_sel_w && !cpu_vwrb_w && cpu_vd_oe_w;
	wire vram1_wren_w = vram1_sel_w && !cpu_vwrb_w && cpu_vd_oe_w;
	wire io_port_active_w = io_cycle_w;
	wire cpu_vram_port_active_w = vram0_sel_w || vram1_sel_w || vram_sel_conflict_w;
	wire savestate_io_sel_w = savestate_mem_active_i && (savestate_mem_type_i == 3'd1);
	wire savestate_vram0_sel_w = savestate_mem_active_i && (savestate_mem_type_i == 3'd2);
	wire savestate_vram1_sel_w = savestate_mem_active_i && (savestate_mem_type_i == 3'd3);
	wire savestate_cpu_sel_w = savestate_mem_active_i && (savestate_mem_type_i == 3'd4);
	wire [7:0] io_ram_addr_w = savestate_io_sel_w ?
		savestate_mem_addr_i[7:0] :
		(io_port_active_w ? cpu_a_w[7:0] : io_ram_addr_q);
	wire [12:0] cpu_vram_addr_hold_w = cpu_vram_port_active_w ? cpu_va_w : cpu_vram_addr_q;
	wire [12:0] vram_port_a_addr_w = (savestate_vram0_sel_w || savestate_vram1_sel_w) ?
		savestate_mem_addr_i[12:0] : cpu_vram_addr_hold_w;
	wire io_ram_wren_mux_w = savestate_io_sel_w ? savestate_mem_wr_i : io_ram_wren_w;
	wire [7:0] io_ram_wdata_mux_w = savestate_io_sel_w ? savestate_mem_wdata_i : cpu_d_dout_w;
	wire vram0_wren_mux_w = savestate_vram0_sel_w ? savestate_mem_wr_i : vram0_wren_w;
	wire vram1_wren_mux_w = savestate_vram1_sel_w ? savestate_mem_wr_i : vram1_wren_w;
	wire [7:0] vram_wdata_mux_w = (savestate_vram0_sel_w || savestate_vram1_sel_w) ?
		savestate_mem_wdata_i : cpu_vd_dout_w;
	// On warm boot, pre-seed the BIOS startup tuple into save RAM before the
	// CPU is released from reset, instead of intercepting later bus traffic.
	wire save_seed_active_w = !reset && warm_boot_i && !warm_boot_seed_done_q;
	wire cpu_reset_w = reset || save_seed_active_w;
	wire [12:0] save_addr_w = save_seed_active_w ?
		(warm_boot_tuple_addr(warm_boot_seed_idx_q)) : cpu_a_w[12:0];
	wire [7:0] save_dout_w = save_seed_active_w ?
		(warm_boot_tuple_data(warm_boot_seed_idx_q)) : cpu_d_dout_w;
	wire save_rd_w = !save_seed_active_w && work_ram_sel_w && !cpu_rdb_w;
	wire save_wren_w = save_seed_active_w ? 1'b1 : work_ram_wren_w;


	assign ext_din_w =
		io_cycle_w ? io_ram_q_w :
		work_ram_sel_w ? save_din_i :
		cart_din_i;

	assign vram_din_w =
		vram0_sel_w ? vram0_cpu_q_w :
		vram1_sel_w ? vram1_cpu_q_w :
		8'h00;

	assign cart_addr_o = cpu_a_w;
	assign cart_dout_o = cpu_d_dout_w;
	assign cart_rd_o = cart_sel_w && !cpu_rdb_w;
	assign cart_wr_o = cart_sel_w && !cpu_wrb_w && cpu_d_oe_w;
	assign cart_doe_o = cart_wr_o;
	assign cart_slot1_sel_o = cart_slot1_sel_w;
	assign cart_slot2_sel_o = cart_slot2_sel_w;
	assign save_addr_o = save_addr_w;
	assign save_dout_o = save_dout_w;
	assign save_rd_o = save_rd_w;
	assign save_wren_o = save_wren_w;

	assign cpu_sound_o = cpu_sound_w;
	assign audio_pcm_o = audio_pcm_w;
	assign cpu_txdb_o = cpu_txdb_w;
	assign cpu_lcd_clk_o = cpu_clk_w;
	assign cpu_doffb_o = cpu_doffb_w;
	assign cpu_p3_din_w = {
		board_p3_din_w[7:4],
		board_p3_din_w[3] & uart_dsr_i,
		board_p3_din_w[2] & uart_cts_i,
		board_p3_din_w[1:0]
	};
	// The Power key still lives in the matrix. Game.com hardware also uses it
	// as a STOP/HALT wake source, but do not drive the SM8521 INTB pin during
	// normal runtime because the Power key is not wired to the external interrupt
	// input. Keep one boot-time press pending until the BIOS reaches STOP so a
	// short front-panel press can complete startup.
	assign cpu_intb_w = 1'b1;
	assign power_cpu_wake_w = power_int_sync1_q || power_stop_wake_q || (cpu_stopped_w && power_boot_pending_q);
	assign uart_dtr_o = cpu_p3_oe_w[0] ? cpu_p3_dout_w[0] : 1'b1;
	assign uart_rts_o = cpu_p3_oe_w[1] ? cpu_p3_dout_w[1] : 1'b1;
	assign savestate_pause_ready_o = cpu_savestate_pause_ready_w;
	assign savestate_mem_rdata_o =
		savestate_io_sel_w ? io_ram_q_w :
		savestate_vram0_sel_w ? vram0_cpu_q_w :
		savestate_vram1_sel_w ? vram1_cpu_q_w :
		savestate_cpu_sel_w ? cpu_savestate_rdata_w :
		8'h00;

	gamecom_audio_output u_audio_output (
		.clk_sys_i(clk_sys),
		.ce_i(phi0),
		.reset_i(cpu_reset_w),
		.sample_i(cpu_sound_w),
		.sample_o(audio_pcm_w)
	);

	function [12:0] warm_boot_tuple_addr;
		input [2:0] idx;
		begin
			warm_boot_tuple_addr = WARM_BOOT_TUPLE_BASE + {10'd0, idx};
		end
	endfunction

	function [7:0] warm_boot_tuple_data;
		input [2:0] idx;
		begin
			case (idx)
				3'd0: warm_boot_tuple_data = 8'h01;
				3'd1: warm_boot_tuple_data = 8'h09;
				3'd2: warm_boot_tuple_data = 8'h09;
				3'd3: warm_boot_tuple_data = 8'h07;
				3'd4: warm_boot_tuple_data = 8'h00;
				default: warm_boot_tuple_data = 8'h04;
			endcase
		end
	endfunction

`ifndef SYNTHESIS
	always @(posedge clk_sys) begin
		if (!reset && vram_sel_conflict_w) begin
			$display("FATAL gamecom: simultaneous VRAM page select va=%04h vrdb=%0d vwrb=%0d vd_oe=%0d",
				{3'b000, cpu_va_w},
				cpu_vrdb_w,
				cpu_vwrb_w,
				cpu_vd_oe_w);
			$finish;
		end
		if (!reset && rom_cycle_w && sram_cycle_w) begin
			$display("FATAL gamecom: simultaneous MCE0B/MCE1B a=%05h rdb=%0d wrb=%0d doe=%0d",
				cpu_a_w,
				cpu_rdb_w,
				cpu_wrb_w,
				cpu_d_oe_w);
			$finish;
		end
		if (!reset && sram_cycle_w && ((cpu_a_w[20:16] != 5'h00) || (cpu_a_w[15:13] != 3'b111))) begin
			$display("FATAL gamecom: SRAM-class cycle outside E000-FFFF a=%05h", cpu_a_w);
			$finish;
		end
	end
`endif

	always @(posedge clk_sys) begin
		if (reset) begin
			warm_boot_seed_idx_q <= 3'd0;
			warm_boot_seed_done_q <= !warm_boot_i;
			io_ram_addr_q <= 8'h00;
			cpu_vram_addr_q <= 13'h0000;
			power_int_sync0_q <= 1'b0;
			power_int_sync1_q <= 1'b0;
			power_stop_wake_q <= 1'b0;
			power_boot_pending_q <= 1'b0;
			power_boot_armed_q <= 1'b1;
		end else begin
			power_int_sync0_q <= button_power_i;
			power_int_sync1_q <= power_int_sync0_q;
			if (power_stop_wake_q && !cpu_stopped_w) begin
				power_stop_wake_q <= 1'b0;
				power_boot_pending_q <= 1'b0;
				power_boot_armed_q <= 1'b0;
			end else if (power_boot_armed_q && power_int_sync1_q) begin
				power_boot_pending_q <= 1'b1;
			end
			if (cpu_stopped_w && (power_int_sync1_q || power_boot_pending_q)) begin
				power_stop_wake_q <= 1'b1;
			end
			if (!warm_boot_i) begin
				warm_boot_seed_idx_q <= 3'd0;
				warm_boot_seed_done_q <= 1'b1;
			end else if (!warm_boot_seed_done_q) begin
				if (warm_boot_seed_idx_q == WARM_BOOT_TUPLE_LAST_IDX) begin
					warm_boot_seed_idx_q <= 3'd0;
					warm_boot_seed_done_q <= 1'b1;
				end else begin
					warm_boot_seed_idx_q <= warm_boot_seed_idx_q + 3'd1;
				end
			end
			if (io_port_active_w) begin
				io_ram_addr_q <= cpu_a_w[7:0];
			end
			if (cpu_vram_port_active_w) begin
				cpu_vram_addr_q <= cpu_va_w;
			end
		end
	end

	gamecom_input u_input (
		.clk_sys_i(clk_sys),
		.ce_i(phi0),
		.reset_i(reset),
		.p0_dout_i(cpu_p0_dout_w),
		.p0_oe_i(cpu_p0_oe_w),
		.p0_pullup_i(cpu_p0_pullup_w),
		.p1_dout_i(cpu_p1_dout_w),
		.p1_oe_i(cpu_p1_oe_w),
		.p1_pullup_i(cpu_p1_pullup_w),
		.p2_dout_i(cpu_p2_dout_w),
		.p2_oe_i(cpu_p2_oe_w),
		.p2_pullup_i(cpu_p2_pullup_w),
		.p3_dout_i(cpu_p3_dout_w),
		.p3_oe_i(cpu_p3_oe_w),
		.p3_pullup_i(cpu_p3_pullup_w),
		.p3_latch_i(cpu_p3_latch_w[7:6]),
		.scan_update_i(cpu_pio_scan_update_w),
		.button_up_i(button_up_i),
		.button_down_i(button_down_i),
		.button_left_i(button_left_i),
		.button_right_i(button_right_i),
		.button_menu_i(button_menu_i),
		.button_sound_i(button_sound_i),
		.button_a_i(button_a_i),
		.button_b_i(button_b_i),
		.button_c_i(button_c_i),
		.button_d_i(button_d_i),
		.button_pause_i(button_pause_i),
		.button_power_i(button_power_i),
		.touch_active_i(touch_active_i),
		.touch_x_i(touch_x_i),
		.touch_y_i(touch_y_i),
		.p0_din_o(board_p0_din_w),
		.p1_din_o(board_p1_din_w),
		.p2_din_o(board_p2_din_w),
		.p3_din_o(board_p3_din_w),
		.slot1_sel_o(cart_slot1_sel_w),
		.slot2_sel_o(cart_slot2_sel_w)
	);

	sm8521 u_cpu (
		.clk_sys_i(clk_sys),
		.phi0_ce_i(phi0),
		.phi1_ce_i(phi1),
		.resetb_i(~cpu_reset_w),
		.stop_disable_i(stop_disable_i),
		.warm_boot_i(warm_boot_i),
		.rtc_i(rtc_i),
		.nmib_i(1'b1),
		.intb_i(cpu_intb_w),
		.power_stop_wake_i(power_cpu_wake_w),
		.m_i(3'b000),
		.d_din_i(ext_din_w),
		// Not a real SM8521 pin: this is the MiSTer SDRAM-backed ROM
		// "READY" helper. Tie high for original fixed-ROM behavior.
		.rom_read_ready_i(rom_read_ready_i),
		.a_o(cpu_a_w),
		.d_dout_o(cpu_d_dout_w),
		.d_oe_o(cpu_d_oe_w),
		.mce0b_o(cpu_mce0b_w),
		.mce1b_o(cpu_mce1b_w),
		.ioe0b_o(cpu_ioe0b_w),
		.ioe1b_o(cpu_ioe1b_w),
		.rdb_o(cpu_rdb_w),
		.wrb_o(cpu_wrb_w),
		.vd_din_i(vram_din_w),
		.va_o(cpu_va_w),
		.vd_dout_o(cpu_vd_dout_w),
		.vd_oe_o(cpu_vd_oe_w),
		.vce0b_o(cpu_vce0b_w),
		.vce1b_o(cpu_vce1b_w),
		.vrdb_o(cpu_vrdb_w),
		.vwrb_o(cpu_vwrb_w),
		.p0_din_i(board_p0_din_w),
		.p0_dout_o(cpu_p0_dout_w),
		.p0_oe_o(cpu_p0_oe_w),
		.p0_pullup_o(cpu_p0_pullup_w),
		.p1_din_i(board_p1_din_w),
		.p1_dout_o(cpu_p1_dout_w),
		.p1_oe_o(cpu_p1_oe_w),
		.p1_pullup_o(cpu_p1_pullup_w),
		.p2_din_i(board_p2_din_w),
		.p2_dout_o(cpu_p2_dout_w),
		.p2_oe_o(cpu_p2_oe_w),
		.p2_pullup_o(cpu_p2_pullup_w),
		.p3_din_i(cpu_p3_din_w),
		.p3_dout_o(cpu_p3_dout_w),
		.p3_oe_o(cpu_p3_oe_w),
		.p3_pullup_o(cpu_p3_pullup_w),
		.p3_latch_o(cpu_p3_latch_w),
		.pio_scan_update_o(cpu_pio_scan_update_w),
		.rxdb_i(uart_rxd_i),
		.vr_i(video_native_vblank_w),
		.txdb_o(cpu_txdb_w),
		.sound_o(cpu_sound_w),
		.fr_o(),
		.lp_o(),
		.xc_o(),
		.xd_o(),
		.yd_o(),
		.display_page_o(cpu_display_page_w),
		.display_palette_o(cpu_display_palette_w),
		.display_normal_black_o(cpu_display_normal_black_w),
		.display_hdot_200_o(),
		.display_vlines_o(),
		.display_dma_active_o(),
		.stopped_o(cpu_stopped_w),
		.doffb_o(cpu_doffb_w),
		.clk_o(cpu_clk_w),
		.savestate_pause_req_i(savestate_pause_req_i),
		.savestate_pause_ready_o(cpu_savestate_pause_ready_w),
		.savestate_active_i(savestate_cpu_sel_w),
		.savestate_addr_i(savestate_mem_addr_i[11:0]),
		.savestate_rd_i(savestate_mem_rd_i && savestate_cpu_sel_w),
		.savestate_wr_i(savestate_mem_wr_i && savestate_cpu_sel_w),
		.savestate_wdata_i(savestate_mem_wdata_i),
		.cheat_clear_i(cheat_clear_i),
		.cheat_code_i(cheat_code_i),
		.savestate_rdata_o(cpu_savestate_rdata_w)
	);

	cache_ram #(
		.ADDR_WIDTH(8),
		.DATA_WIDTH(8)
	) u_io_stub (
		.clk_i(clk_sys),
		.addr_i(io_ram_addr_w),
		.wren_i(io_ram_wren_mux_w),
		.wdata_i(io_ram_wdata_mux_w),
		.q_o(io_ram_q_w)
	);

	cache_ram_dp #(
		.ADDR_WIDTH(13),
		.DATA_WIDTH(8),
		// The MiSTer scanout path reads VRAM continuously on port B while the
		// CPU/DMA updates it on port A. Forward same-address A->B collisions so
		// scanout sees the just-written byte instead of an undefined stale/new
		// mix from the dual-port BRAM wrapper.
		.CROSS_PORT_FORWARD(1'b1)
	) u_vram0 (
		.clk_i(clk_sys),
		.addr_a_i(vram_port_a_addr_w),
		.wren_a_i(vram0_wren_mux_w),
		.wdata_a_i(vram_wdata_mux_w),
		.q_a_o(vram0_cpu_q_w),
		.addr_b_i(video_vram_addr_w),
		.wren_b_i(1'b0),
		.wdata_b_i(8'h00),
		.q_b_o(vram0_video_q_w)
	);

	cache_ram_dp #(
		.ADDR_WIDTH(13),
		.DATA_WIDTH(8),
		.CROSS_PORT_FORWARD(1'b1)
	) u_vram1 (
		.clk_i(clk_sys),
		.addr_a_i(vram_port_a_addr_w),
		.wren_a_i(vram1_wren_mux_w),
		.wdata_a_i(vram_wdata_mux_w),
		.q_a_o(vram1_cpu_q_w),
		.addr_b_i(video_vram_addr_w),
		.wren_b_i(1'b0),
		.wdata_b_i(8'h00),
		.q_b_o(vram1_video_q_w)
	);

	gamecom_video #(.VIDEO_CLOCK_DIV(VIDEO_CLOCK_DIV), .STOP_LOGO_FILE(STOP_LOGO_FILE)) u_video (
		.clk_sys_i(clk_sys),
		.ce_pix_i(phi0),
		.clk_vid_i(clk_vid),
		.reset_i(video_reset_i),
		.display_enable_i(!reset && cpu_doffb_w),
		.display_page_req_i(cpu_display_page_w),
		.display_palette_i(cpu_display_palette_w),
		.display_normal_black_i(cpu_display_normal_black_w),
		.palette_four_color_i(palette_four_color_i),
		// Game.com exposes a fixed 200x160 host bitmap. LCH[5] and LCV[5:4]
		// remain implemented inside the SM8521 LCDC but do not resize scanout.
		.video_60hz_i(video_60hz_i),
		.stop_mode_i(cpu_stopped_w),
		.cursor_enable_i(cursor_enable_i),
		.cursor_x_i(cursor_x_i),
		.cursor_y_i(cursor_y_i),
		.vram0_din_i(vram0_video_q_w),
		.vram1_din_i(vram1_video_q_w),
		.vram_addr_o(video_vram_addr_w),
		.ce_pix_o(ce_pix),
		.hblank_o(HBlank),
		.hsync_o(HSync),
		.vblank_o(VBlank),
		.native_vblank_o(video_native_vblank_w),
		.vsync_o(VSync),
		.shade_o(shade)
	);

endmodule
