// Copyright (c) 2026 Jamie Blanks

module gamecom_input
(
	input wire clk_sys_i,
	input wire ce_i,
	input wire reset_i,
	input wire [7:0] p0_dout_i,
	input wire [7:0] p0_oe_i,
	input wire [7:0] p0_pullup_i,
	input wire [7:0] p1_dout_i,
	input wire [7:0] p1_oe_i,
	input wire [7:0] p1_pullup_i,
	input wire [7:0] p2_dout_i,
	input wire [7:0] p2_oe_i,
	input wire [7:0] p2_pullup_i,
	input wire [7:0] p3_dout_i,
	input wire [7:0] p3_oe_i,
	input wire [7:0] p3_pullup_i,
	input wire [1:0] p3_latch_i,
	input wire scan_update_i,
	input wire button_up_i,
	input wire button_down_i,
	input wire button_left_i,
	input wire button_right_i,
	input wire button_menu_i,
	input wire button_sound_i,
	input wire button_a_i,
	input wire button_b_i,
	input wire button_c_i,
	input wire button_d_i,
	input wire button_pause_i,
	input wire button_power_i,
	input wire touch_active_i,
	input wire [3:0] touch_x_i,
	input wire [3:0] touch_y_i,

	output reg [7:0] p0_din_o,
	output reg [7:0] p1_din_o,
	output reg [7:0] p2_din_o,
	output reg [7:0] p3_din_o,
	output reg slot1_sel_o,
	output reg slot2_sel_o
);

	localparam [7:0] P0_BOARD_IDLE_HIGH_MASK = 8'hFF;
	localparam [7:0] P1_BOARD_IDLE_HIGH_MASK = 8'hFF;
	localparam [7:0] P2_BOARD_IDLE_HIGH_MASK = 8'hFF;
	localparam [7:0] P3_BOARD_IDLE_HIGH_MASK = 8'hFF;

	function [7:0] resolve_board_pins;
		input [7:0] port_dout;
		input [7:0] port_oe;
		input [7:0] port_pullup;
		input [7:0] board_idle_high;
		begin
			resolve_board_pins = (port_dout & port_oe) |
				((port_pullup | board_idle_high) & ~port_oe);
		end
	endfunction

	// Use the driven pin levels for matrix selection. The datasheet pull-up
	// mode is explicit in p*_pullup_i; the Game.com matrix itself is still
	// modeled as inactive-high when a line is released.
	wire [7:0] p0_pin_base_w = resolve_board_pins(p0_dout_i, p0_oe_i, p0_pullup_i, P0_BOARD_IDLE_HIGH_MASK);
	wire [7:0] p1_scan_w = resolve_board_pins(p1_dout_i, p1_oe_i, p1_pullup_i, P1_BOARD_IDLE_HIGH_MASK);
	wire [7:0] p2_scan_w = resolve_board_pins(p2_dout_i, p2_oe_i, p2_pullup_i, P2_BOARD_IDLE_HIGH_MASK);
	wire [7:0] p3_pin_base_w = resolve_board_pins(p3_dout_i, p3_oe_i, p3_pullup_i, P3_BOARD_IDLE_HIGH_MASK);

	// Match the kernel's Game.com input scan behavior: touch columns use one
	// active-low bit across P1[7:2] and P2[7:0], the main key row is selected
	// by P2[7]=0, and the power/button-D row is the idle all-ones selector.
	wire [13:0] scan_w = {p2_scan_w, p1_scan_w[7:2]};
	wire [9:0] touch_rows_w =
		(touch_active_i && (touch_x_i < 4'd13) && (touch_y_i < 4'd10)) ?
			(10'h3FF & ~(10'h001 << touch_y_i)) :
			10'h3FF;
	wire [7:0] keys1_p0_w = {
		~button_a_i,
		~button_sound_i,
		~button_pause_i,
		~button_menu_i,
		~button_right_i,
		~button_left_i,
		~button_down_i,
		~button_up_i
	};
	wire [7:0] keys1_p1_w = {6'h3F, ~button_c_i, ~button_b_i};
	wire [1:0] keys2_p0_w = {~button_d_i, ~button_power_i};
	reg [7:0] p0_hold_q;
	reg [7:0] p1_hold_q;
	reg scan_update_pending_q;
	reg [7:0] p0_sample_v;
	reg [7:0] p1_sample_v;
	reg [7:0] p2_sample_v;
	reg [7:0] p3_sample_v;

	always @(*) begin
		p0_sample_v = 8'hFF;
		p1_sample_v = 8'hFF;
		p2_sample_v = 8'hFF;
		p3_sample_v = 8'hFF;
		p0_din_o = p0_hold_q;
		p1_din_o = p1_hold_q;
		p2_din_o = 8'hFF;
		p3_din_o = 8'hFF;
		slot1_sel_o = 1'b0;
		slot2_sel_o = 1'b0;

		if (!reset_i) begin
			// MAME updates P0/P1 only for recognized Game.com mux states. For
			// unrecognized selectors, preserve the previous sampled values so
			// transient P1/P2 scans cannot turn into synthetic key events.
			p0_sample_v = p0_hold_q;
			p1_sample_v = p1_hold_q;
			p2_sample_v = p2_scan_w;
			p3_sample_v = p3_pin_base_w;

			// The Game.com cartridge decoder follows the P3 slot-select latch.
			// P37 can be repurposed as the Timer 1 clock output by the generic
			// SM8521 port mode; using that physical pin value here makes slot 2
			// disappear whenever the timer output is low.
			case (p3_latch_i)
				2'b01: begin
					slot1_sel_o = 1'b1;
				end
				2'b10: begin
					slot2_sel_o = 1'b1;
				end
				default: begin
				end
			endcase

			case (scan_w)
				14'h3FFE,
				14'h3FFD,
				14'h3FFB,
				14'h3FF7,
				14'h3FEF,
				14'h3FDF,
				14'h3FBF,
				14'h3F7F,
				14'h3EFF,
				14'h3DFF,
				14'h3BFF,
				14'h37FF,
				14'h2FFF: begin
					p0_sample_v = 8'hFF;
					p1_sample_v = p1_scan_w | 8'h03;
					if (touch_active_i && (touch_x_i < 4'd13) && (scan_w == (14'h3FFF & ~(14'h0001 << touch_x_i)))) begin
						p0_sample_v = p0_pin_base_w & touch_rows_w[7:0];
						p1_sample_v = p1_scan_w & {6'h3F, touch_rows_w[9:8]};
					end
				end
				14'h1FFF: begin
					p0_sample_v = p0_pin_base_w & keys1_p0_w;
					p1_sample_v = p1_scan_w & keys1_p1_w;
				end
				14'h3FFF: begin
					// MAME's Game.com mux updates only power/button-D on the
					// all-ones row; P0[7:2] retain the previous sampled value.
					p0_sample_v = {p0_hold_q[7:2], (p0_pin_base_w[1:0] & keys2_p0_w)};
					p1_sample_v = 8'hFF;
				end
				default: begin
				end
			endcase

			p2_din_o = p2_sample_v;
			p3_din_o = p3_sample_v;
			p0_din_o = p0_hold_q;
			p1_din_o = p1_hold_q;
		end
	end

	always @(posedge clk_sys_i) begin
		if (reset_i) begin
			p0_hold_q <= 8'hFF;
			p1_hold_q <= 8'hFF;
			scan_update_pending_q <= 1'b0;
		end else begin
			if (scan_update_i) begin
				scan_update_pending_q <= 1'b1;
			end
			if (ce_i && (scan_update_i || scan_update_pending_q)) begin
				p0_hold_q <= p0_sample_v;
				p1_hold_q <= p1_sample_v;
				scan_update_pending_q <= 1'b0;
			end
		end
	end

endmodule
