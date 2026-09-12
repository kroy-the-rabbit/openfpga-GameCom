// Game.com SD-card ROM core for Analogue Pocket.
`default_nettype none

module core_top (

//
// physical connections
//

///////////////////////////////////////////////////
// clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

input   wire            clk_74a, // mainclk1
input   wire            clk_74b, // mainclk1

///////////////////////////////////////////////////
// cartridge interface
// switches between 3.3v and 5v mechanically
// output enable for multibit translators controlled by pic32

// GBA AD[15:8]
inout   wire    [7:0]   cart_tran_bank2,
output  wire            cart_tran_bank2_dir,

// GBA AD[7:0]
inout   wire    [7:0]   cart_tran_bank3,
output  wire            cart_tran_bank3_dir,

// GBA A[23:16]
inout   wire    [7:0]   cart_tran_bank1,
output  wire            cart_tran_bank1_dir,

// GBA [7] PHI#
// GBA [6] WR#
// GBA [5] RD#
// GBA [4] CS1#/CS#
//     [3:0] unwired
inout   wire    [7:4]   cart_tran_bank0,
output  wire            cart_tran_bank0_dir,

// GBA CS2#/RES#
inout   wire            cart_tran_pin30,
output  wire            cart_tran_pin30_dir,
output  wire            cart_pin30_pwroff_reset,

// GBA IRQ/DRQ
inout   wire            cart_tran_pin31,
output  wire            cart_tran_pin31_dir,

// infrared
input   wire            port_ir_rx,
output  wire            port_ir_tx,
output  wire            port_ir_rx_disable,

// GBA link port
inout   wire            port_tran_si,
output  wire            port_tran_si_dir,
inout   wire            port_tran_so,
output  wire            port_tran_so_dir,
inout   wire            port_tran_sck,
output  wire            port_tran_sck_dir,
inout   wire            port_tran_sd,
output  wire            port_tran_sd_dir,

///////////////////////////////////////////////////
// cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

output  wire    [21:16] cram0_a,
inout   wire    [15:0]  cram0_dq,
input   wire            cram0_wait,
output  wire            cram0_clk,
output  wire            cram0_adv_n,
output  wire            cram0_cre,
output  wire            cram0_ce0_n,
output  wire            cram0_ce1_n,
output  wire            cram0_oe_n,
output  wire            cram0_we_n,
output  wire            cram0_ub_n,
output  wire            cram0_lb_n,

output  wire    [21:16] cram1_a,
inout   wire    [15:0]  cram1_dq,
input   wire            cram1_wait,
output  wire            cram1_clk,
output  wire            cram1_adv_n,
output  wire            cram1_cre,
output  wire            cram1_ce0_n,
output  wire            cram1_ce1_n,
output  wire            cram1_oe_n,
output  wire            cram1_we_n,
output  wire            cram1_ub_n,
output  wire            cram1_lb_n,

///////////////////////////////////////////////////
// sdram, 512mbit 16bit

output  wire    [12:0]  dram_a,
output  wire    [1:0]   dram_ba,
inout   wire    [15:0]  dram_dq,
output  wire    [1:0]   dram_dqm,
output  wire            dram_clk,
output  wire            dram_cke,
output  wire            dram_ras_n,
output  wire            dram_cas_n,
output  wire            dram_we_n,

///////////////////////////////////////////////////
// sram, 1mbit 16bit

output  wire    [16:0]  sram_a,
inout   wire    [15:0]  sram_dq,
output  wire            sram_oe_n,
output  wire            sram_we_n,
output  wire            sram_ub_n,
output  wire            sram_lb_n,

///////////////////////////////////////////////////
// vblank driven by dock for sync in a certain mode

input   wire            vblank,

///////////////////////////////////////////////////
// i/o to 6515D breakout usb uart

output  wire            dbg_tx,
input   wire            dbg_rx,

///////////////////////////////////////////////////
// i/o pads near jtag connector user can solder to

output  wire            user1,
input   wire            user2,

///////////////////////////////////////////////////
// RFU internal i2c bus

inout   wire            aux_sda,
output  wire            aux_scl,

///////////////////////////////////////////////////
// RFU, do not use
output  wire            vpll_feed,


//
// logical connections
//

///////////////////////////////////////////////////
// video, audio output to scaler
output  wire    [23:0]  video_rgb,
output  wire            video_rgb_clock,
output  wire            video_rgb_clock_90,
output  wire            video_de,
output  wire            video_skip,
output  wire            video_vs,
output  wire            video_hs,

output  wire            audio_mclk,
input   wire            audio_adc,
output  wire            audio_dac,
output  wire            audio_lrck,

///////////////////////////////////////////////////
// bridge bus connection
// synchronous to clk_74a
output  wire            bridge_endian_little,
input   wire    [31:0]  bridge_addr,
input   wire            bridge_rd,
output  reg     [31:0]  bridge_rd_data,
input   wire            bridge_wr,
input   wire    [31:0]  bridge_wr_data,

///////////////////////////////////////////////////
// controller data
//
// key bitmap:
//   [0]    dpad_up
//   [1]    dpad_down
//   [2]    dpad_left
//   [3]    dpad_right
//   [4]    face_a
//   [5]    face_b
//   [6]    face_x
//   [7]    face_y
//   [8]    trig_l1
//   [9]    trig_r1
//   [10]   trig_l2
//   [11]   trig_r2
//   [12]   trig_l3
//   [13]   trig_r3
//   [14]   face_select
//   [15]   face_start
//   [31:28] type
// joy values - unsigned
//   [ 7: 0] lstick_x
//   [15: 8] lstick_y
//   [23:16] rstick_x
//   [31:24] rstick_y
// trigger values - unsigned
//   [ 7: 0] ltrig
//   [15: 8] rtrig
//
input   wire    [31:0]  cont1_key,
input   wire    [31:0]  cont2_key,
input   wire    [31:0]  cont3_key,
input   wire    [31:0]  cont4_key,
input   wire    [31:0]  cont1_joy,
input   wire    [31:0]  cont2_joy,
input   wire    [31:0]  cont3_joy,
input   wire    [31:0]  cont4_joy,
input   wire    [15:0]  cont1_trig,
input   wire    [15:0]  cont2_trig,
input   wire    [15:0]  cont3_trig,
input   wire    [15:0]  cont4_trig

);

// The ROM-only core never enables a physical cartridge or link transceiver.
assign cart_tran_bank0 = 4'hf;
assign cart_tran_bank0_dir = 1'b1;
assign cart_tran_bank1 = 8'hzz;
assign cart_tran_bank2 = 8'hzz;
assign cart_tran_bank3 = 8'hzz;
assign cart_tran_bank1_dir = 1'b0;
assign cart_tran_bank2_dir = 1'b0;
assign cart_tran_bank3_dir = 1'b0;
assign cart_tran_pin30 = 1'bz;
assign cart_tran_pin31 = 1'bz;
assign cart_tran_pin30_dir = 1'b0;
assign cart_tran_pin31_dir = 1'b0;
assign cart_pin30_pwroff_reset = 1'b0;
assign port_tran_si = 1'bz;
assign port_tran_so = 1'bz;
assign port_tran_sck = 1'bz;
assign port_tran_sd = 1'bz;
assign port_tran_si_dir = 1'b0;
assign port_tran_so_dir = 1'b0;
assign port_tran_sck_dir = 1'b0;
assign port_tran_sd_dir = 1'b0;
assign port_ir_tx = 1'b0;
assign port_ir_rx_disable = 1'b1;
assign cram0_a = 6'd0;
assign cram1_a = 6'd0;
assign cram0_dq = 16'hzzzz;
assign cram1_dq = 16'hzzzz;
assign cram0_clk = 1'b0;
assign cram1_clk = 1'b0;
assign cram0_cre = 1'b0;
assign cram1_cre = 1'b0;
assign {cram0_adv_n,cram0_ce0_n,cram0_ce1_n,cram0_oe_n,cram0_we_n,cram0_ub_n,cram0_lb_n} = 7'h7f;
assign {cram1_adv_n,cram1_ce0_n,cram1_ce1_n,cram1_oe_n,cram1_we_n,cram1_ub_n,cram1_lb_n} = 7'h7f;
assign dbg_tx = 1'bz;
assign user1 = 1'bz;
assign aux_sda = 1'bz;
assign aux_scl = 1'bz;
assign vpll_feed = 1'bz;
assign bridge_endian_little = 1'b0;

wire clk_mem, clk_sdram, clk_sys, clk_vid, clk_vid_90, pll_locked;
reg [7:0] pll_startup = 8'hff;
always @(posedge clk_74a) if (pll_startup != 0) pll_startup <= pll_startup - 1'b1;
wire pll_reset = pll_startup != 0;
gamecom_pll machine_pll (
    .refclk(clk_74a), .rst(pll_reset), .clk_mem(clk_mem), .clk_sdram(clk_sdram),
    .clk_sys(clk_sys), .clk_vid(clk_vid), .clk_vid_90(clk_vid_90), .locked(pll_locked)
);
wire clk_audio, audio_locked;
mf_audio_pll audio_pll (
    .refclk(clk_74a), .rst(pll_reset), .outclk_0(audio_mclk),
    .outclk_1(clk_audio), .locked(audio_locked)
);
wire cold_reset = !pll_locked || !audio_locked;
wire reset_bridge, reset_mem, reset_sys, reset_audio;
gamecom_reset_sync rb(.clk(clk_74a),.async_reset(cold_reset),.reset(reset_bridge));
gamecom_reset_sync rm(.clk(clk_mem),.async_reset(cold_reset),.reset(reset_mem));
gamecom_reset_sync rs(.clk(clk_sys),.async_reset(cold_reset),.reset(reset_sys));
gamecom_reset_sync ra(.clk(clk_audio),.async_reset(cold_reset),.reset(reset_audio));
assign video_rgb_clock = clk_vid;
assign video_rgb_clock_90 = clk_vid_90;

wire apf_reset_n, request_write, request_read, request_ack, request_ok, request_reject;
wire [15:0] request_id;
wire [31:0] request_size, cmd_rd_data;
wire allcomplete, assets_ready, bios_loaded, cart_loaded, loader_busy;
wire [21:0] rom_size;
wire [7:0] loader_error;
wire [31:0] announced_bytes, received_bytes, committed_bytes, readback_crc32, input_crc32;
wire [6:0] fifo_high_water;
wire [3:0] loader_phase;
wire memory_initialized, memory_fault, ram_initialized;
wire rtc_valid;
wire [31:0] rtc_date_bcd, rtc_time_bcd;

(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg [2:0] memory_ready_b = 0, ram_ready_b = 0, running_b = 0;
(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg [2:0] allowed_s = 0;
reg [21:0] rom_size_s = 0;
reg reset_toggle = 0, power_toggle = 0;
(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg [2:0] reset_action_s = 0, power_action_s = 0;
reg reset_seen = 0, power_seen = 0;
reg [5:0] reset_hold = 0;
wire ready_to_run = assets_ready && ram_ready_b[2];
wire run_reset = reset_sys || !allowed_s[2] || reset_hold != 0;
wire power_pulse = power_action_s[2] != power_seen;
always @(posedge clk_74a) begin
    memory_ready_b <= {memory_ready_b[1:0],memory_initialized};
    ram_ready_b <= {ram_ready_b[1:0],ram_initialized};
    running_b <= {running_b[1:0],!run_reset};
    if (reset_bridge) begin
        memory_ready_b <= 0; ram_ready_b <= 0; running_b <= 0;
        reset_toggle <= 0; power_toggle <= 0;
    end else if (bridge_wr) begin
        if (bridge_addr == 32'hf0000000 && bridge_wr_data[0]) reset_toggle <= ~reset_toggle;
        if (bridge_addr == 32'hf0000004 && bridge_wr_data[0]) power_toggle <= ~power_toggle;
    end
end
always @(posedge clk_sys) begin
    allowed_s <= {allowed_s[1:0],ready_to_run && apf_reset_n};
    reset_action_s <= {reset_action_s[1:0],reset_toggle};
    power_action_s <= {power_action_s[1:0],power_toggle};
    power_seen <= power_action_s[2];
    reset_seen <= reset_action_s[2];
    // Asset metadata is held stable before allowed_s can rise.
    if (!allowed_s[2]) rom_size_s <= rom_size;
    if (reset_hold != 0) reset_hold <= reset_hold - 1'b1;
    if (reset_action_s[2] != reset_seen) reset_hold <= 6'd63;
    if (reset_sys) begin
        allowed_s <= 0; rom_size_s <= 0;
        reset_action_s <= 0; power_action_s <= 0;
        reset_seen <= 0; power_seen <= 0; reset_hold <= 0;
    end
end

core_bridge_cmd command_handler (
    .clk(clk_74a), .reset_n(apf_reset_n),
    .bridge_endian_little(bridge_endian_little), .bridge_addr(bridge_addr),
    .bridge_rd(bridge_rd), .bridge_rd_data(cmd_rd_data),
    .bridge_wr(bridge_wr), .bridge_wr_data(bridge_wr_data),
    .status_boot_done(!reset_bridge && memory_ready_b[2]),
    .status_setup_done(ready_to_run), .status_running(running_b[2]),
    .dataslot_requestread(request_read), .dataslot_requestread_id(),
    .dataslot_requestread_ack(request_read), .dataslot_requestread_ok(1'b0),
    .dataslot_requestwrite(request_write), .dataslot_requestwrite_id(request_id),
    .dataslot_requestwrite_size(request_size), .dataslot_requestwrite_ack(request_ack),
    .dataslot_requestwrite_ok(request_ok), .dataslot_requestwrite_reject(request_reject),
    .dataslot_update(), .dataslot_update_id(), .dataslot_update_size(),
    .dataslot_allcomplete(allcomplete), .rtc_epoch_seconds(),
    .rtc_date_bcd(rtc_date_bcd), .rtc_time_bcd(rtc_time_bcd), .rtc_valid(rtc_valid),
    .savestate_supported(1'b0), .savestate_addr(32'd0), .savestate_size(32'd0),
    .savestate_maxloadsize(32'd0), .osnotify_inmenu(),
    .osnotify_cart_play(), .osnotify_cart_power(),
    .savestate_start(), .savestate_start_ack(1'b0), .savestate_start_busy(1'b0),
    .savestate_start_ok(1'b0), .savestate_start_err(1'b0),
    .savestate_load(), .savestate_load_ack(1'b0), .savestate_load_busy(1'b0),
    .savestate_load_ok(1'b0), .savestate_load_err(1'b0),
    .target_dataslot_read(1'b0), .target_dataslot_write(1'b0),
    .target_dataslot_getfile(1'b0), .target_dataslot_openfile(1'b0),
    .target_dataslot_ack(), .target_dataslot_done(), .target_dataslot_err(),
    .target_dataslot_id(16'd0), .target_dataslot_slotoffset(32'd0),
    .target_dataslot_bridgeaddr(32'd0), .target_dataslot_length(32'd0),
    .target_buffer_param_struct(32'd0), .target_buffer_resp_struct(32'd0),
    .datatable_addr(10'd0), .datatable_wren(1'b0), .datatable_data(32'd0), .datatable_q()
);

wire load_valid, load_ready, load_read, load_bios, load_done;
wire memory_load_ready;
(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg [2:0] load_allowed_m = 0;
always @(posedge clk_mem) begin
    load_allowed_m <= {load_allowed_m[1:0],run_reset};
    if (reset_mem) load_allowed_m <= 0;
end
// Hold the CPU in reset while loading and verifying assets.
assign load_ready = memory_load_ready && load_allowed_m[2];
wire [20:0] load_addr;
wire [31:0] load_data, load_rdata;
gamecom_rom_loader loader (
    .clk_bridge(clk_74a), .reset_bridge(reset_bridge), .clk_mem(clk_mem), .reset_mem(reset_mem),
    .bridge_wr(bridge_wr), .bridge_addr(bridge_addr), .bridge_wr_data(bridge_wr_data),
    .dataslot_requestwrite(request_write), .dataslot_requestwrite_id(request_id),
    .dataslot_requestwrite_size(request_size), .dataslot_ack(request_ack),
    .dataslot_ok(request_ok), .dataslot_reject(request_reject), .dataslot_allcomplete(allcomplete),
    .memory_initialized(memory_initialized), .memory_fault(memory_fault),
    .load_valid(load_valid), .load_ready(load_ready), .load_bios(load_bios), .load_read(load_read),
    .load_addr(load_addr), .load_data(load_data), .load_done(load_done), .load_rdata(load_rdata),
    .assets_ready(assets_ready), .bios_loaded(bios_loaded), .cart_loaded(cart_loaded),
    .rom_size(rom_size), .busy(loader_busy), .error(loader_error), .phase(loader_phase),
    .announced_bytes(announced_bytes), .received_bytes(received_bytes),
    .committed_bytes(committed_bytes), .readback_crc32(readback_crc32),
    .input_crc32(input_crc32), .fifo_high_water(fifo_high_water)
);
wire [20:0] cart_addr;
wire [7:0] cart_data;
wire cart_rd, slot1_sel, slot2_sel, rom_read_ready;
gamecom_memory memory (
    .clk_mem(clk_mem), .clk_sdram(clk_sdram), .reset_mem(reset_mem), .clk_sys(clk_sys), .reset_sys(run_reset),
    .load_valid(load_valid && load_allowed_m[2]), .load_ready(memory_load_ready), .load_read(load_read), .load_bios(load_bios),
    .load_addr(load_addr), .load_data(load_data), .load_done(load_done), .load_rdata(load_rdata),
    .initialized(memory_initialized), .fault(memory_fault),
    .cart_addr(cart_addr), .cart_rd(cart_rd), .slot1_sel(slot1_sel), .slot2_sel(slot2_sel),
    .rom_size(rom_size_s), .cart_present(allowed_s[2]), .cart_data(cart_data), .rom_read_ready(rom_read_ready),
    .dram_a(dram_a), .dram_ba(dram_ba), .dram_dq(dram_dq), .dram_dqm(dram_dqm),
    .dram_clk(dram_clk), .dram_cke(dram_cke), .dram_ras_n(dram_ras_n),
    .dram_cas_n(dram_cas_n), .dram_we_n(dram_we_n),
    .sram_a(sram_a), .sram_dq(sram_dq), .sram_oe_n(sram_oe_n), .sram_we_n(sram_we_n),
    .sram_ub_n(sram_ub_n), .sram_lb_n(sram_lb_n)
);
wire signed [15:0] audio_sample;
gamecom_machine console (
    .clk_sys(clk_sys), .clk_vid(clk_vid), .clk_bridge(clk_74a),
    .reset_cold(reset_sys), .reset_run(run_reset), .cont1_key(cont1_key), .power_pulse(power_pulse),
    .rtc_valid(rtc_valid), .rtc_date_bcd(rtc_date_bcd), .rtc_time_bcd(rtc_time_bcd),
    .cart_data(cart_data), .rom_ready(rom_read_ready), .ram_initialized(ram_initialized),
    .cart_addr(cart_addr), .cart_rd(cart_rd), .cart_slot1_sel(slot1_sel), .cart_slot2_sel(slot2_sel),
    .video_rgb(video_rgb), .video_hs(video_hs), .video_vs(video_vs), .video_de(video_de), .video_skip(video_skip),
    .audio_sample(audio_sample)
);
gamecom_i2s audio_transport (
    .clk_sys(clk_sys), .reset_sys(reset_sys), .sample(audio_sample),
    .clk_audio(clk_audio), .reset_audio(reset_audio), .audio_lrck(audio_lrck), .audio_dac(audio_dac)
);

// All diagnostic values are published by the loader in the bridge domain.
always @* begin
    bridge_rd_data = 32'd0;
    if (bridge_addr[31:24] == 8'hf8) bridge_rd_data = cmd_rd_data;
    else case (bridge_addr)
        32'hf4000000: bridge_rd_data = {20'd0,loader_phase,3'd0,loader_busy,running_b[2],assets_ready,cart_loaded,bios_loaded};
        32'hf4000004: bridge_rd_data = {24'd0,loader_error};
        32'hf4000008: bridge_rd_data = announced_bytes;
        32'hf400000c: bridge_rd_data = received_bytes;
        32'hf4000010: bridge_rd_data = committed_bytes;
        32'hf4000014: bridge_rd_data = readback_crc32;
        32'hf4000018: bridge_rd_data = {25'd0,fifo_high_water};
        32'hf400001c: bridge_rd_data = {10'd0,rom_size};
        32'hf4000020: bridge_rd_data = input_crc32;
        default: bridge_rd_data = 0;
    endcase
end
endmodule
`default_nettype wire
