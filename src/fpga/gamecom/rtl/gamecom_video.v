// Copyright (c) 2026 Jamie Blanks

module gamecom_video
#(
	parameter integer VIDEO_CLOCK_DIV = 10,
	parameter STOP_LOGO_FILE = "rtl/gamecom_stop_logo.hex"
)
(
	input wire clk_sys_i,
	input wire ce_pix_i,
	input wire clk_vid_i,
	input wire reset_i,
	input wire display_enable_i,
	input wire display_page_req_i,
	input wire [1:0] display_palette_i,
	input wire display_normal_black_i,
	input wire palette_four_color_i,
	input wire video_60hz_i,
	input wire stop_mode_i,
	input wire cursor_enable_i,
	input wire [3:0] cursor_x_i,
	input wire [3:0] cursor_y_i,
	input wire [7:0] vram0_din_i,
	input wire [7:0] vram1_din_i,
	output wire [12:0] vram_addr_o,
	output wire ce_pix_o,
	output wire hblank_o,
	output wire hsync_o,
	output wire vblank_o,
	output wire native_vblank_o,
	output wire vsync_o,
	output wire [2:0] shade_o
);

	// Keep the MiSTer-visible adapter on the default SM8521 LCDC readout
	// cadence: 40 byte shifts * 8 LCH phases by 200+7 LCD lines.
	localparam [8:0] H_TOTAL = 9'd320;
	localparam [8:0] H_SYNC = 9'd24;
	localparam [8:0] H_START = 9'd72;

	localparam [8:0] V_TOTAL = 9'd207;
	localparam [8:0] V_SYNC = 9'd1;
	localparam [8:0] V_START = 9'd24;

	// The 60 Hz raster runs on clk_vid/10 = 6.000 MHz so that 200 dots occupy
	// 33.33 us of the NTSC line. That is what makes the dots square on a 4:3
	// analog display; the 5 MHz CPU pixel rate would stretch them 21% wide.
	localparam [8:0] TV60_H_TOTAL = 9'd381;
	localparam [8:0] TV60_H_SYNC = 9'd28;
	// A TV centres on the standard NTSC active window, which runs 9.4 us to
	// 62.06 us after the sync edge and so is centred at 35.73 us. Putting the
	// 33.33 us image there starts it at 19.06 us. Balancing the porches
	// instead would centre it in the total line and sit 1.7 us left on a tube.
	localparam [8:0] TV60_H_START = 9'd114;
	localparam [8:0] TV60_V_TOTAL = 9'd262;
	localparam [8:0] TV60_V_SYNC = 9'd3;
	localparam [8:0] TV60_V_START = 9'd52;

	localparam [7:0] FRAME_W = 8'd200;
	localparam [7:0] FRAME_H = 8'd160;

	reg [8:0] h_count_q;
	reg [8:0] v_count_q;
	reg [12:0] vram_addr_q;
	reg display_page_q;
	reg display_enable_q;
	reg [1:0] display_palette_q;
	reg display_normal_black_q;
	reg palette_four_color_q;
	reg fetch_valid_q;
	reg fetch_stop_q;
	reg fetch_page_q;
	reg [1:0] fetch_pix_idx_q;
	reg fetch_cursor_q;
	reg hblank_q;
	reg hsync_q;
	reg vblank_q;
	reg vsync_q;
	reg [2:0] shade_q;
	reg [3:0] vid_div_q;
	reg phi0_vid_q;
	reg phi0_vid_d_q;
	reg vid60_s1_q;
	reg vid60_s2_q;
	reg nat_hblank_q;
	reg nat_hsync_q;
	reg nat_vblank_q;
	reg nat_vsync_q;
	reg [2:0] nat_shade_q;
	reg fb_wr_q;
	reg [14:0] fb_waddr_q;
	reg [2:0] fb_wdata_q;
	reg fb_wbuf_q;
	reg cap_done_tgl_q;
	reg cap_done_buf_q;
	reg cap_tgl_s1_q;
	reg cap_tgl_s2_q;
	reg cap_buf_s1_q;
	reg cap_buf_s2_q;
	reg disp_seen_tgl_q;
	reg disp_ack_s1_q;
	reg disp_ack_s2_q;
	reg [8:0] tv60_h_count_q;
	reg [8:0] tv60_v_count_q;
	reg [14:0] fb_read_addr_q;
	reg tv60_fetch_valid_q;
	reg tv60_hblank_q;
	reg tv60_hsync_q;
	reg tv60_vblank_q;
	reg tv60_vsync_q;
	reg [2:0] tv60_shade_q;
	reg [7:0] capture_x_q;
	reg [7:0] capture_y_q;
	reg capture_active_q;
	reg capture_prev_vblank_q;
	reg tv60_prev_vblank_q;
	reg fb_write_buf_q;
	reg fb_display_buf_q;
	reg fb_display_valid_q;

	localparam [7:0] POWER_LOGO_W = 8'd128;
	localparam [7:0] POWER_LOGO_H = 8'd96;

	wire [8:0] h_active_pixels_w = {1'b0, FRAME_W};
	wire [8:0] h_end_w = H_START + h_active_pixels_w;
	wire [8:0] v_active_lines_w = {1'b0, FRAME_H};
	wire [8:0] v_end_w = V_START + v_active_lines_w;
	wire [7:0] selected_vram_data_w = fetch_page_q ? vram1_din_i : vram0_din_i;
	wire [7:0] logo_din_w;
	wire [7:0] selected_pixel_data_w = fetch_stop_q ? logo_din_w : selected_vram_data_w;
	wire render_active_w = display_enable_q || stop_mode_i;
	wire h_active_w = (h_count_q >= H_START) && (h_count_q < h_end_w);
	wire v_active_w = (v_count_q >= V_START) && (v_count_q < v_end_w);
	wire line_end_w = (h_count_q == (H_TOTAL - 9'd1));
	wire frame_end_w = line_end_w && (v_count_q == (V_TOTAL - 9'd1));
	wire line_start_w = (h_count_q == 9'd0);
	wire [8:0] next_h_count_w = line_end_w ? 9'd0 : (h_count_q + 9'd1);
	wire [8:0] next_v_count_w = frame_end_w ? 9'd0 : (line_end_w ? (v_count_q + 9'd1) : v_count_q);
	wire next_h_active_w = (next_h_count_w >= H_START) && (next_h_count_w < h_end_w);
	wire next_v_active_w = (next_v_count_w >= V_START) && (next_v_count_w < v_end_w);
	wire [7:0] next_active_x_w = next_h_count_w[7:0] - H_START[7:0];
	wire [7:0] next_active_y_w = next_v_count_w[7:0] - V_START[7:0];
	wire [7:0] cursor_center_x_w = cursor_x_center(cursor_x_i);
	wire [7:0] cursor_center_y_w = cursor_y_center(cursor_y_i);
	wire next_cursor_vline_w = (next_active_x_w >= (cursor_center_x_w - 8'd1)) && (next_active_x_w <= (cursor_center_x_w + 8'd1)) &&
		(next_active_y_w >= (cursor_center_y_w - 8'd8)) && (next_active_y_w <= (cursor_center_y_w + 8'd8));
	wire next_cursor_hline_w = (next_active_y_w >= (cursor_center_y_w - 8'd1)) && (next_active_y_w <= (cursor_center_y_w + 8'd1)) &&
		(next_active_x_w >= (cursor_center_x_w - 8'd8)) && (next_active_x_w <= (cursor_center_x_w + 8'd8));
	wire next_cursor_hit_w = cursor_enable_i && (next_cursor_vline_w || next_cursor_hline_w);
	wire [7:0] power_logo_left_w = power_logo_left(h_active_pixels_w);
	wire [7:0] power_logo_top_w = power_logo_top(v_active_lines_w);
	wire next_logo_hit_w = power_logo_hit(next_active_x_w, next_active_y_w, h_active_pixels_w, v_active_lines_w);
	wire [7:0] next_logo_x_w = next_active_x_w - power_logo_left_w;
	wire [7:0] next_logo_y_w = next_active_y_w - power_logo_top_w;
	wire [11:0] next_logo_addr_w = power_logo_addr(next_logo_x_w, next_logo_y_w);
	wire [11:0] logo_addr_w = next_logo_hit_w ? next_logo_addr_w : 12'd0;
	wire [1:0] raw_pixel_shade_w = pixel_shade(selected_pixel_data_w, fetch_pix_idx_q);
	wire [2:0] lcd_shade_fwd_w = palette_four_color_q ?
		four_color_shade(raw_pixel_shade_w) :
		lcd_gradation_shade(raw_pixel_shade_w, display_palette_q);
	// LCC[0] picks the panel's rest state. Normal black flips the drive sense,
	// so the same levels come out mirrored down the ladder: white swaps with
	// black and the greys swap in pairs, leaving the middle grey alone. The
	// five-pen gradation palette mirrors about pen 2. The four-colour host
	// palette is not a hardware mode; it lives in entries 1 to 4 with entry 0
	// held back for blanking, so it mirrors about the middle of that range.
	wire [2:0] lcd_shade_inv_w = palette_four_color_q ?
		(3'd5 - lcd_shade_fwd_w) : (3'd4 - lcd_shade_fwd_w);
	wire [2:0] lcd_pixel_shade_w = display_normal_black_q ?
		lcd_shade_inv_w : lcd_shade_fwd_w;
	wire tv60_line_end_w = (tv60_h_count_q == (TV60_H_TOTAL - 9'd1));
	wire tv60_frame_end_w = tv60_line_end_w && (tv60_v_count_q == (TV60_V_TOTAL - 9'd1));
	wire [8:0] tv60_next_h_count_w = tv60_line_end_w ? 9'd0 : (tv60_h_count_q + 9'd1);
	wire [8:0] tv60_next_v_count_w = tv60_frame_end_w ? 9'd0 : (tv60_line_end_w ? (tv60_v_count_q + 9'd1) : tv60_v_count_q);
	wire tv60_h_active_w = (tv60_h_count_q >= TV60_H_START) && (tv60_h_count_q < (TV60_H_START + 9'd200));
	wire tv60_v_active_w = (tv60_v_count_q >= TV60_V_START) && (tv60_v_count_q < (TV60_V_START + 9'd160));
	wire tv60_next_h_active_w = (tv60_next_h_count_w >= TV60_H_START) && (tv60_next_h_count_w < (TV60_H_START + 9'd200));
	wire tv60_next_v_active_w = (tv60_next_v_count_w >= TV60_V_START) && (tv60_next_v_count_w < (TV60_V_START + 9'd160));
	wire [7:0] tv60_next_x_w = tv60_next_h_count_w[7:0] - TV60_H_START[7:0];
	wire [7:0] tv60_next_y_w = tv60_next_v_count_w[7:0] - TV60_V_START[7:0];
	// clk_sys and clk_vid rise together, so this pulse lands two clk_vid edges
	// after the phi0 edge and the clk_sys registers it produced have settled.
	wire ce_native_w = phi0_vid_q && !phi0_vid_d_q;
	wire ce_tv60_w = (vid_div_q == 4'd0);
	wire ce_out_w = vid60_s2_q ? ce_tv60_w : ce_native_w;
	// The capture side may reuse a buffer once the display side has taken it.
	wire fb_pending_w = (cap_done_tgl_q != disp_ack_s2_q);
	wire fb_swap_w = (cap_tgl_s2_q != disp_seen_tgl_q);
	wire fb_wren_w = ce_native_w && fb_wr_q;
	wire native_frame_start_w = capture_prev_vblank_q && !vblank_q;
	wire native_frame_end_w = !capture_prev_vblank_q && vblank_q;
	wire tv60_vblank_start_w = !tv60_prev_vblank_q && tv60_vblank_q;
	wire capture_start_w = video_60hz_i && native_frame_start_w && !fb_pending_w;
	wire capture_in_frame_w = capture_active_q || capture_start_w;
	wire capture_visible_w = !hblank_q && !vblank_q;
	wire capture_wren_w = video_60hz_i && capture_in_frame_w && capture_visible_w &&
		(capture_x_q < FRAME_W) && (capture_y_q < FRAME_H);
	wire [14:0] capture_addr_w = frame_addr(capture_x_q, capture_y_q);
	wire [14:0] tv60_read_next_addr_w = frame_addr(tv60_next_x_w, tv60_next_y_w);
	wire [2:0] fb0_q_w;
	wire [2:0] fb1_q_w;
	wire [2:0] fb_read_data_w = fb_display_buf_q ? fb1_q_w : fb0_q_w;

	function [7:0] cursor_x_center;
		input [3:0] cursor_x;
		begin
			// MAME's Game.com touch layout has twelve 16-pixel columns,
			// then an 8-pixel right-edge column (192..199).
			if (cursor_x < 4'd12) cursor_x_center = {cursor_x, 4'b1000};
			else cursor_x_center = 8'd196;
		end
	endfunction

	function [7:0] cursor_y_center;
		input [3:0] cursor_y;
		begin
			case (cursor_y)
				4'd0: cursor_y_center = 8'd8;
				4'd1: cursor_y_center = 8'd24;
				4'd2: cursor_y_center = 8'd40;
				4'd3: cursor_y_center = 8'd56;
				4'd4: cursor_y_center = 8'd72;
				4'd5: cursor_y_center = 8'd88;
				4'd6: cursor_y_center = 8'd104;
				4'd7: cursor_y_center = 8'd120;
				4'd8: cursor_y_center = 8'd136;
				default: cursor_y_center = 8'd152;
			endcase
		end
	endfunction

	function [12:0] vram_scan_addr;
		input [7:0] out_x_i;
		input [7:0] out_y_i;
		reg [12:0] column_base_v;
		begin
			// The host bitmap is rotated from the fixed 40-byte VRAM row.
			column_base_v = ({5'h00, out_x_i} << 5) + ({5'h00, out_x_i} << 3);
			vram_scan_addr = column_base_v + {7'h00, out_y_i[7:2]};
		end
	endfunction

	function [14:0] frame_addr;
		input [7:0] out_x_i;
		input [7:0] out_y_i;
		reg [14:0] row_base_v;
		begin
			row_base_v = ({7'h00, out_y_i} << 7) +
				({7'h00, out_y_i} << 6) +
				({7'h00, out_y_i} << 3);
			frame_addr = row_base_v + {7'h00, out_x_i};
		end
	endfunction

	function [1:0] vram_scan_pix_idx;
		input [7:0] out_y_i;
		begin
			vram_scan_pix_idx = out_y_i[1:0];
		end
	endfunction

	function [1:0] pixel_shade;
		input [7:0] src_byte_i;
		input [1:0] pix_idx_i;
		begin
			case (pix_idx_i)
				2'd0: pixel_shade = src_byte_i[7:6];
				2'd1: pixel_shade = src_byte_i[5:4];
				2'd2: pixel_shade = src_byte_i[3:2];
				default: pixel_shade = src_byte_i[1:0];
			endcase
		end
	endfunction

	function [2:0] lcd_gradation_shade;
		input [1:0] dot_i;
		input [1:0] gradation_i;
		begin
			// Match MAME's five-entry Game.com LCD palette exactly:
			// pen 0 = black, 1 = gray1, 2 = gray2, 3 = gray3, 4 = white.
			//
			// Careful: settings 00 and 11 are swapped against the SM8521
			// datasheet, which pairs 00 with gray1/gray2 and 11 with
			// gray2/gray3. The palettes were picked so that 11 - the setting
			// the BIOS and nearly every game use - comes out as an even
			// brightness ramp, which only works with the swap in place.
			// Correcting this table alone washes out almost every title; the
			// table and the palette pens have to move together.
			case (gradation_i)
				2'b01,
				2'b10: begin
					case (dot_i)
						2'd0: lcd_gradation_shade = 3'd4;
						2'd1: lcd_gradation_shade = 3'd3;
						2'd2: lcd_gradation_shade = 3'd1;
						default: lcd_gradation_shade = 3'd0;
					endcase
				end
				2'b11: begin
					case (dot_i)
						2'd0: lcd_gradation_shade = 3'd4;
						2'd1: lcd_gradation_shade = 3'd2;
						2'd2: lcd_gradation_shade = 3'd1;
						default: lcd_gradation_shade = 3'd0;
					endcase
				end
				default: begin
					case (dot_i)
						2'd0: lcd_gradation_shade = 3'd4;
						2'd1: lcd_gradation_shade = 3'd3;
						2'd2: lcd_gradation_shade = 3'd2;
						default: lcd_gradation_shade = 3'd0;
					endcase
				end
			endcase
		end
	endfunction

	function [2:0] four_color_shade;
		input [1:0] dot_i;
		begin
			// A four-color palette occupies entries one through four. Keep
			// entry five (shade 0) reserved for blanking/display-off black.
			case (dot_i)
				2'd0: four_color_shade = 3'd4;
				2'd1: four_color_shade = 3'd3;
				2'd2: four_color_shade = 3'd2;
				default: four_color_shade = 3'd1;
			endcase
		end
	endfunction

	function [7:0] power_logo_left;
		input [8:0] active_w_i;
		begin
			if (active_w_i <= {1'b0, POWER_LOGO_W}) begin
				power_logo_left = 8'd0;
			end else begin
				power_logo_left = (active_w_i[7:0] - POWER_LOGO_W) >> 1;
			end
		end
	endfunction

	function [7:0] power_logo_top;
		input [8:0] active_h_i;
		begin
			if (active_h_i <= {1'b0, POWER_LOGO_H}) begin
				power_logo_top = 8'd0;
			end else begin
				power_logo_top = (active_h_i[7:0] - POWER_LOGO_H) >> 1;
			end
		end
	endfunction

	function power_logo_hit;
		input [7:0] out_x_i;
		input [7:0] out_y_i;
		input [8:0] active_w_i;
		input [8:0] active_h_i;
		reg [7:0] left_v;
		reg [7:0] top_v;
		begin
			left_v = power_logo_left(active_w_i);
			top_v = power_logo_top(active_h_i);
			power_logo_hit = (out_x_i >= left_v) && (out_x_i < (left_v + POWER_LOGO_W)) &&
				(out_y_i >= top_v) && (out_y_i < (top_v + POWER_LOGO_H));
		end
	endfunction

	function [11:0] power_logo_addr;
		input [7:0] logo_x_i;
		input [7:0] logo_y_i;
		reg [11:0] row_base_v;
		begin
			row_base_v = {logo_y_i[6:0], 5'b00000};
			power_logo_addr = row_base_v + {5'b00000, logo_x_i[7:2]};
		end
	endfunction

	function [1:0] power_logo_pix_idx;
		input [7:0] logo_x_i;
		begin
			power_logo_pix_idx = logo_x_i[1:0] + 2'd1;
		end
	endfunction

	assign vram_addr_o = vram_addr_q;
	assign ce_pix_o = ce_out_w;
	assign hblank_o = vid60_s2_q ? tv60_hblank_q : nat_hblank_q;
	assign hsync_o = vid60_s2_q ? tv60_hsync_q : nat_hsync_q;
	assign vblank_o = vid60_s2_q ? tv60_vblank_q : nat_vblank_q;
	assign native_vblank_o = vblank_q;
	assign vsync_o = vid60_s2_q ? tv60_vsync_q : nat_vsync_q;
	assign shade_o = vid60_s2_q ? tv60_shade_q : nat_shade_q;

	gamecom_stop_logo_rom #(.STOP_LOGO_FILE(STOP_LOGO_FILE)) u_power_logo_rom (
		.clk_sys_i(clk_sys_i),
		.ce_i(ce_pix_i),
		.addr_i(logo_addr_w),
		.q_o(logo_din_w)
	);

	cache_ram_dp #(
		.ADDR_WIDTH(15),
		.DATA_WIDTH(3),
		.CROSS_PORT_FORWARD(1'b0)
	) u_framebuf0 (
		.clk_i(clk_vid_i),
		.addr_a_i(fb_waddr_q),
		.wren_a_i(fb_wren_w && !fb_wbuf_q),
		.wdata_a_i(fb_wdata_q),
		.q_a_o(),
		.addr_b_i(fb_read_addr_q),
		.wren_b_i(1'b0),
		.wdata_b_i(3'd0),
		.q_b_o(fb0_q_w)
	);

	cache_ram_dp #(
		.ADDR_WIDTH(15),
		.DATA_WIDTH(3),
		.CROSS_PORT_FORWARD(1'b0)
	) u_framebuf1 (
		.clk_i(clk_vid_i),
		.addr_a_i(fb_waddr_q),
		.wren_a_i(fb_wren_w && fb_wbuf_q),
		.wdata_a_i(fb_wdata_q),
		.q_a_o(),
		.addr_b_i(fb_read_addr_q),
		.wren_b_i(1'b0),
		.wdata_b_i(3'd0),
		.q_b_o(fb1_q_w)
	);

	always @(posedge clk_sys_i) begin
		if (reset_i) begin
			h_count_q <= 9'd0;
			v_count_q <= 9'd0;
			vram_addr_q <= 13'd0;
			display_page_q <= 1'b0;
			display_enable_q <= 1'b0;
			display_palette_q <= 2'b00;
			display_normal_black_q <= 1'b0;
			palette_four_color_q <= 1'b0;
			fetch_valid_q <= 1'b0;
			fetch_stop_q <= 1'b0;
			fetch_page_q <= 1'b0;
			fetch_pix_idx_q <= 2'b00;
			fetch_cursor_q <= 1'b0;
			hblank_q <= 1'b1;
			hsync_q <= 1'b1;
			vblank_q <= 1'b1;
			vsync_q <= 1'b1;
			shade_q <= 3'd0;
			capture_x_q <= 8'd0;
			capture_y_q <= 8'd0;
			capture_active_q <= 1'b0;
			capture_prev_vblank_q <= 1'b1;
			fb_write_buf_q <= 1'b1;
			cap_done_tgl_q <= 1'b0;
			cap_done_buf_q <= 1'b1;
			disp_ack_s1_q <= 1'b0;
			disp_ack_s2_q <= 1'b0;
			fb_wr_q <= 1'b0;
			fb_waddr_q <= 15'd0;
			fb_wdata_q <= 3'd0;
			fb_wbuf_q <= 1'b1;
		end else begin
			disp_ack_s1_q <= disp_seen_tgl_q;
			disp_ack_s2_q <= disp_ack_s1_q;

			if (ce_pix_i) begin
				// VRAM runs on clk_sys and settles during the fabric clocks between
				// pixel enables. The prior pixel's fetch metadata therefore lines up
				// directly with the current RAM output.
				hblank_q <= !h_active_w;
				hsync_q <= (h_count_q < H_SYNC);
				vblank_q <= !v_active_w;
				vsync_q <= (v_count_q < V_SYNC);

				if (line_start_w) begin
					// MAME samples LCDC display state at scanline granularity.
					// Keep page, enable, gradation, inversion, and the host
					// palette shape stable so a control change cannot splice two
					// LCDC states through one MiSTer-visible scanline.
					display_page_q <= display_page_req_i;
					display_enable_q <= display_enable_i;
					display_palette_q <= display_palette_i;
					display_normal_black_q <= display_normal_black_i;
					palette_four_color_q <= palette_four_color_i;
				end

				if (!h_active_w || !v_active_w) begin
					shade_q <= 3'd0;
				end else if (fetch_stop_q) begin
					if (fetch_valid_q) begin
						shade_q <= lcd_pixel_shade_w;
					end else begin
						shade_q <= 3'd4;
					end
				end else if (fetch_valid_q) begin
					if (fetch_cursor_q) begin
						shade_q <= (lcd_pixel_shade_w >= 3'd3) ? 3'd0 : 3'd4;
					end else begin
						shade_q <= lcd_pixel_shade_w;
					end
				end else begin
					// Blank pipeline bubbles before the first fetched byte.
					shade_q <= 3'd0;
				end

				if (next_h_active_w && next_v_active_w && render_active_w) begin
					fetch_stop_q <= stop_mode_i;
					fetch_page_q <= display_page_q;
					if (stop_mode_i) begin
						fetch_cursor_q <= 1'b0;
						if (next_logo_hit_w) begin
							fetch_pix_idx_q <= power_logo_pix_idx(next_logo_x_w);
							fetch_valid_q <= 1'b1;
						end else begin
							fetch_pix_idx_q <= 2'b00;
							fetch_valid_q <= 1'b0;
						end
					end else begin
						vram_addr_q <= vram_scan_addr(next_active_x_w, next_active_y_w);
						// Latch the page alongside the fetch so the returned synchronous
						// BRAM byte cannot be paired with a later page selection change.
						fetch_pix_idx_q <= vram_scan_pix_idx(next_active_y_w);
						fetch_cursor_q <= next_cursor_hit_w;
						fetch_valid_q <= 1'b1;
					end
				end else begin
					fetch_page_q <= display_page_q;
					fetch_stop_q <= stop_mode_i;
					fetch_pix_idx_q <= 2'b00;
					fetch_cursor_q <= 1'b0;
					fetch_valid_q <= 1'b0;
				end

				if (line_end_w) begin
					h_count_q <= 9'd0;
					if (v_count_q == (V_TOTAL - 9'd1)) begin
						v_count_q <= 9'd0;
					end else begin
						v_count_q <= v_count_q + 9'd1;
					end
				end else begin
					h_count_q <= h_count_q + 9'd1;
				end

				// Frame capture for the 60 Hz reader. The reader lives in the
				// video clock, so the write is handed over one pixel later
				// through fb_w*_q rather than driving the RAM from here.
				capture_prev_vblank_q <= vblank_q;
				fb_wr_q <= capture_wren_w;
				fb_waddr_q <= capture_addr_w;
				fb_wdata_q <= shade_q;
				fb_wbuf_q <= fb_write_buf_q;

				if (!video_60hz_i) begin
					capture_x_q <= 8'd0;
					capture_y_q <= 8'd0;
					capture_active_q <= 1'b0;
					fb_write_buf_q <= 1'b1;
					cap_done_tgl_q <= 1'b0;
					cap_done_buf_q <= 1'b1;
				end else if (native_frame_start_w) begin
					capture_x_q <= 8'd0;
					capture_y_q <= 8'd0;
					capture_active_q <= !fb_pending_w;
				end else if (native_frame_end_w) begin
					if (capture_active_q) begin
						// Hand this buffer to the reader and take the other one.
						cap_done_tgl_q <= ~cap_done_tgl_q;
						cap_done_buf_q <= fb_write_buf_q;
						fb_write_buf_q <= ~fb_write_buf_q;
					end
					capture_active_q <= 1'b0;
					capture_x_q <= 8'd0;
					capture_y_q <= 8'd0;
				end else if (capture_wren_w) begin
					if (capture_x_q == (FRAME_W - 8'd1)) begin
						capture_x_q <= 8'd0;
						if (capture_y_q != (FRAME_H - 8'd1)) begin
							capture_y_q <= capture_y_q + 8'd1;
						end
					end else begin
						capture_x_q <= capture_x_q + 8'd1;
					end
				end else if (vblank_q) begin
					capture_x_q <= 8'd0;
					capture_y_q <= 8'd0;
				end
			end
		end
	end

	always @(posedge clk_vid_i) begin
		if (reset_i) begin
			vid_div_q <= 4'd0;
			phi0_vid_q <= 1'b0;
			phi0_vid_d_q <= 1'b0;
			vid60_s1_q <= 1'b0;
			vid60_s2_q <= 1'b0;
			cap_tgl_s1_q <= 1'b0;
			cap_tgl_s2_q <= 1'b0;
			cap_buf_s1_q <= 1'b1;
			cap_buf_s2_q <= 1'b1;
			disp_seen_tgl_q <= 1'b0;
			nat_hblank_q <= 1'b1;
			nat_hsync_q <= 1'b1;
			nat_vblank_q <= 1'b1;
			nat_vsync_q <= 1'b1;
			nat_shade_q <= 3'd0;
			tv60_h_count_q <= 9'd0;
			tv60_v_count_q <= 9'd0;
			fb_read_addr_q <= 15'd0;
			tv60_fetch_valid_q <= 1'b0;
			tv60_hblank_q <= 1'b1;
			tv60_hsync_q <= 1'b1;
			tv60_vblank_q <= 1'b1;
			tv60_vsync_q <= 1'b1;
			tv60_shade_q <= 3'd0;
			tv60_prev_vblank_q <= 1'b1;
			fb_display_buf_q <= 1'b0;
			fb_display_valid_q <= 1'b0;
		end else begin
			phi0_vid_q <= ce_pix_i;
			phi0_vid_d_q <= phi0_vid_q;
			vid60_s1_q <= video_60hz_i;
			vid60_s2_q <= vid60_s1_q;
			cap_tgl_s1_q <= cap_done_tgl_q;
			cap_tgl_s2_q <= cap_tgl_s1_q;
			cap_buf_s1_q <= cap_done_buf_q;
			cap_buf_s2_q <= cap_buf_s1_q;
			vid_div_q <= (vid_div_q == (VIDEO_CLOCK_DIV[3:0] - 4'd1)) ? 4'd0 : (vid_div_q + 4'd1);

			// The native raster is produced at the CPU pixel rate; only its
			// outputs are retimed here so the framework sees one clock.
			if (ce_native_w) begin
				nat_hblank_q <= hblank_q;
				nat_hsync_q <= hsync_q;
				nat_vblank_q <= vblank_q;
				nat_vsync_q <= vsync_q;
				nat_shade_q <= shade_q;
			end

			if (ce_tv60_w) begin
				if (!vid60_s2_q) begin
					tv60_h_count_q <= 9'd0;
					tv60_v_count_q <= 9'd0;
					fb_read_addr_q <= 15'd0;
					tv60_fetch_valid_q <= 1'b0;
					tv60_hblank_q <= 1'b1;
					tv60_hsync_q <= 1'b1;
					tv60_vblank_q <= 1'b1;
					tv60_vsync_q <= 1'b1;
					tv60_shade_q <= 3'd0;
					tv60_prev_vblank_q <= 1'b1;
					disp_seen_tgl_q <= 1'b0;
					fb_display_buf_q <= 1'b0;
					fb_display_valid_q <= 1'b0;
				end else begin
					tv60_prev_vblank_q <= tv60_vblank_q;

					// Take a finished frame only between displayed frames.
					if (tv60_vblank_start_w && fb_swap_w) begin
						fb_display_buf_q <= cap_buf_s2_q;
						disp_seen_tgl_q <= cap_tgl_s2_q;
						fb_display_valid_q <= 1'b1;
					end

					// The frame buffer runs on clk_vid, so its output is ready
					// before the next pixel enable just like native VRAM.
					tv60_hblank_q <= !tv60_h_active_w;
					tv60_hsync_q <= (tv60_h_count_q < TV60_H_SYNC);
					tv60_vblank_q <= !tv60_v_active_w;
					tv60_vsync_q <= (tv60_v_count_q < TV60_V_SYNC);

					if (!tv60_h_active_w || !tv60_v_active_w || !tv60_fetch_valid_q || !fb_display_valid_q) begin
						tv60_shade_q <= 3'd0;
					end else begin
						tv60_shade_q <= fb_read_data_w;
					end

					if (tv60_next_h_active_w && tv60_next_v_active_w && fb_display_valid_q) begin
						fb_read_addr_q <= tv60_read_next_addr_w;
						tv60_fetch_valid_q <= 1'b1;
					end else begin
						tv60_fetch_valid_q <= 1'b0;
					end

					if (tv60_line_end_w) begin
						tv60_h_count_q <= 9'd0;
						if (tv60_v_count_q == (TV60_V_TOTAL - 9'd1)) begin
							tv60_v_count_q <= 9'd0;
						end else begin
							tv60_v_count_q <= tv60_v_count_q + 9'd1;
						end
					end else begin
						tv60_h_count_q <= tv60_h_count_q + 9'd1;
					end
				end
			end
		end
	end

endmodule

module gamecom_stop_logo_rom
#(parameter STOP_LOGO_FILE = "rtl/gamecom_stop_logo.hex")
(
	input wire clk_sys_i,
	input wire ce_i,
	input wire [11:0] addr_i,
	output wire [7:0] q_o
);

	localparam integer MEM_DEPTH = 4096;
	localparam [11:0] LAST_INIT_ADDR = 12'd3071;

	reg [7:0] q_q;
	(* ramstyle = "M10K, no_rw_check" *) reg [7:0] mem_q [0:MEM_DEPTH-1];
	integer init_i;

	initial begin
		for (init_i = 0; init_i < MEM_DEPTH; init_i = init_i + 1) begin
			mem_q[init_i] = 8'h00;
		end
		$readmemh(STOP_LOGO_FILE, mem_q, 0, LAST_INIT_ADDR);
	end

	always @(posedge clk_sys_i) begin
		if (ce_i) begin
			q_q <= mem_q[addr_i];
		end
	end

	assign q_o = q_q;

endmodule
