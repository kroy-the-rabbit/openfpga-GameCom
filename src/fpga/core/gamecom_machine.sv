module gamecom_machine #(
    parameter STOP_LOGO_FILE = "../gamecom/rtl/gamecom_stop_logo.hex"
) (
    input wire clk_sys,       // 20 MHz, phi0/phi1 are 5 MHz enables
    input wire clk_vid,       // 30 MHz; buffered output pixel enable is 6 MHz
    input wire clk_bridge,
    input wire reset_cold,    // FPGA/PLL reset; clears volatile console RAM
    input wire reset_run,     // APF/reset action; preserves console RAM
    input wire [31:0] cont1_key,
    input wire power_pulse,   // clk_sys domain
    input wire rtc_valid,    // remaining rtc_* inputs are clk_bridge domain
    input wire [31:0] rtc_date_bcd,
    input wire [31:0] rtc_time_bcd,
    input wire [7:0] cart_data,
    input wire rom_ready,
    output reg ram_initialized,
    output wire [20:0] cart_addr,
    output wire cart_rd,
    output wire cart_slot1_sel,
    output wire cart_slot2_sel,
    output wire [23:0] video_rgb,
    output wire video_hs, video_vs, video_de, video_skip,
    output wire signed [15:0] audio_sample
);
    reg [1:0] phi_div;
    reg has_run;
    reg [4:0] warm_hold;
    reg [12:0] clear_addr, save_addr_hold;
    wire machine_reset = reset_cold || reset_run || !ram_initialized;
    wire warm_boot = has_run && (reset_run || (warm_hold != 0));
    wire [12:0] save_addr;
    wire [7:0] save_dout, save_din;
    wire save_rd, save_wren;
    wire [12:0] ram_addr = !ram_initialized ? clear_addr :
                          (save_rd || save_wren) ? save_addr : save_addr_hold;
    wire ram_wren = !reset_cold && (!ram_initialized || (!reset_run && save_wren));

    always @(posedge clk_sys) begin
        if (reset_cold) begin
            phi_div <= 0;
            clear_addr <= 0;
            ram_initialized <= 0;
            save_addr_hold <= 0;
            has_run <= 0;
            warm_hold <= 0;
        end else begin
            phi_div <= phi_div + 1'b1;
            if (!ram_initialized) begin
                if (clear_addr == 13'h1FFF) ram_initialized <= 1;
                else clear_addr <= clear_addr + 1'b1;
            end
            if (!machine_reset) has_run <= 1;
            if (reset_run && has_run) warm_hold <= 5'd31;
            else if (warm_hold != 0) warm_hold <= warm_hold - 1'b1;
            if (save_rd || save_wren) save_addr_hold <= save_addr;
        end
    end

    cache_ram #(.ADDR_WIDTH(13), .DATA_WIDTH(8)) console_ram (
        .clk_i(clk_sys), .addr_i(ram_addr), .wren_i(ram_wren),
        .wdata_i(ram_initialized ? save_dout : 8'd0), .q_o(save_din)
    );

    wire [11:0] buttons;
    wire touch_active, cursor_enable;
    wire [3:0] touch_x, touch_y;
    gamecom_pocket_input input_adapter (
        .clk_sys(clk_sys), .reset(reset_cold), .cont1_key(cont1_key),
        .power_pulse(power_pulse), .buttons(buttons),
        .touch_active(touch_active), .cursor_enable(cursor_enable),
        .touch_x(touch_x), .touch_y(touch_y)
    );
    wire [64:0] rtc_bus;
    gamecom_rtc rtc_adapter (
        .clk_bridge(clk_bridge), .clk_sys(clk_sys), .reset_cold(reset_cold),
        .reset_run(machine_reset), .rtc_valid(rtc_valid),
        .rtc_date_bcd(rtc_date_bcd), .rtc_time_bcd(rtc_time_bcd), .rtc_bus(rtc_bus)
    );

    wire ce_pix, hblank, vblank, hsync, vsync;
    wire [2:0] shade;
    wire [15:0] audio_unsigned;
    GameCom #(.VIDEO_CLOCK_DIV(5), .STOP_LOGO_FILE(STOP_LOGO_FILE)) machine (
        .clk_sys(clk_sys), .phi0(phi_div == 0), .phi1(phi_div == 2),
        .clk_vid(clk_vid), .reset(machine_reset), .video_reset_i(machine_reset),
        .stop_disable_i(1'b1), .warm_boot_i(warm_boot),
        .cart_din_i(cart_data), .rom_read_ready_i(rom_ready),
        .uart_rxd_i(1'b1), .uart_cts_i(1'b1), .uart_dsr_i(1'b1),
        .buttons_i(buttons), .touch_active_i(touch_active),
        .touch_x_i(touch_x), .touch_y_i(touch_y),
        .video_60hz_i(1'b1), .palette_four_color_i(1'b0),
        .cursor_enable_i(cursor_enable), .cursor_x_i(touch_x), .cursor_y_i(touch_y),
        .save_din_i(save_din), .rtc_i(rtc_bus),
        .savestate_pause_req_i(1'b0), .savestate_mem_active_i(1'b0),
        .savestate_mem_type_i(3'd0), .savestate_mem_addr_i(25'd0),
        .savestate_mem_rd_i(1'b0), .savestate_mem_wr_i(1'b0),
        .savestate_mem_wdata_i(8'd0), .cheat_clear_i(1'b1), .cheat_code_i(129'd0),
        .ce_pix(ce_pix), .HBlank(hblank), .HSync(hsync),
        .VBlank(vblank), .VSync(vsync), .shade(shade),
        .cart_addr_o(cart_addr), .cart_rd_o(cart_rd),
        .cart_slot1_sel_o(cart_slot1_sel), .cart_slot2_sel_o(cart_slot2_sel),
        .save_addr_o(save_addr), .save_dout_o(save_dout),
        .save_rd_o(save_rd), .save_wren_o(save_wren), .audio_pcm_o(audio_unsigned),
        .cart_dout_o(), .cart_doe_o(), .cart_wr_o(), .cpu_sound_o(), .cpu_txdb_o(),
        .cpu_lcd_clk_o(), .cpu_doffb_o(), .uart_rts_o(), .uart_dtr_o(),
        .savestate_pause_ready_o(), .savestate_mem_rdata_o()
    );
    gamecom_video_adapter video_adapter (
        .clk_vid(clk_vid), .reset(machine_reset), .ce_pix(ce_pix),
        .hblank(hblank), .vblank(vblank), .hsync(hsync), .vsync(vsync), .shade(shade),
        .video_rgb(video_rgb), .video_hs(video_hs), .video_vs(video_vs),
        .video_de(video_de), .video_skip(video_skip)
    );
    assign audio_sample = machine_reset ? 16'sd0 : {~audio_unsigned[15], audio_unsigned[14:0]};
endmodule
