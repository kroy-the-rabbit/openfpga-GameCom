// Copyright (c) 2026 Jamie Blanks

module sm8521_timers
#(
	// The real Game.com core runs phi0 at 5 MHz, and TM0/TM1 are modeled
	// directly in those phi0 ticks. GUESS: CLKT is also counted from this
	// parameterized phi0 interval even though the datasheet derives CLKT from
	// the 32.768 kHz sub-clock.
	parameter integer CLKT_1S_PHI0_CYCLES = 5000000
)
(
	input wire clk_sys_i,
	input wire resetb_i,
	input wire core_reset_i,
	input wire phi0_ce_i,
	input wire main_stopped_i,
	input wire [7:0] tm0c_i,
	input wire [7:0] tm0_reload_i,
	input wire [7:0] tm1c_i,
	input wire [7:0] tm1_reload_i,
	input wire [1:0] tm0c_write_seq_i,
	input wire [1:0] tm0_reload_write_seq_i,
	input wire [1:0] tm1c_write_seq_i,
	input wire [1:0] tm1_reload_write_seq_i,
	input wire [1:0] clkt_write_seq_i,
	input wire clkt_run_i,
	input wire clkt_minute_i,
	output reg [7:0] tm0d_q,
	output reg [15:0] tm0_div_q,
	output reg tm0_out_q,
	output reg [7:0] tm1d_q,
	output reg [15:0] tm1_div_q,
	output reg tm1_out_q,
	output reg [5:0] clkt_count_q,
	output reg [23:0] clkt_div_q,
	output wire tm0_irq_event_o,
	output wire tm1_irq_event_o,
	output wire clkt_second_event_o,
	output wire clk_irq_event_o,
	input wire [4:0] ss_addr_i,
	input wire ss_wren_i,
	input wire [7:0] ss_wdata_i,
	output reg [7:0] ss_rdata_o
);
	reg [1:0] tm0c_write_seq_prev_q;
	reg [1:0] tm0_reload_write_seq_prev_q;
	reg [1:0] tm1c_write_seq_prev_q;
	reg [1:0] tm1_reload_write_seq_prev_q;
	reg [1:0] clkt_write_seq_prev_q;
	reg clkt_run_prev_q;
	reg clkt_minute_prev_q;
	reg tm0_irq_event_v;
	reg tm1_irq_event_v;
	reg clkt_second_event_v;
	reg clk_irq_event_v;

	localparam [23:0] CLKT_1S_PHI0_CYCLES_W = CLKT_1S_PHI0_CYCLES[23:0];

	assign tm0_irq_event_o = tm0_irq_event_v;
	assign tm1_irq_event_o = tm1_irq_event_v;
	assign clkt_second_event_o = clkt_second_event_v;
	assign clk_irq_event_o = clk_irq_event_v;

	always @(*) begin
		case (ss_addr_i)
			5'd0: ss_rdata_o = tm0d_q;
			5'd1: ss_rdata_o = tm0_div_q[7:0];
			5'd2: ss_rdata_o = tm0_div_q[15:8];
			5'd3: ss_rdata_o = {7'b0000000, tm0_out_q};
			5'd4: ss_rdata_o = tm1d_q;
			5'd5: ss_rdata_o = tm1_div_q[7:0];
			5'd6: ss_rdata_o = tm1_div_q[15:8];
			5'd7: ss_rdata_o = {7'b0000000, tm1_out_q};
			5'd8: ss_rdata_o = {2'b00, clkt_count_q};
			5'd9: ss_rdata_o = clkt_div_q[7:0];
			5'd10: ss_rdata_o = clkt_div_q[15:8];
			5'd11: ss_rdata_o = clkt_div_q[23:16];
			5'd12: ss_rdata_o = {
				tm1_reload_write_seq_prev_q,
				tm1c_write_seq_prev_q,
				tm0_reload_write_seq_prev_q,
				tm0c_write_seq_prev_q
			};
			5'd13: ss_rdata_o = {4'b0000, clkt_minute_prev_q, clkt_run_prev_q, clkt_write_seq_prev_q};
			default: ss_rdata_o = 8'h00;
		endcase
	end

	function [15:0] timer_prescale_reload;
		input [2:0] sel;
		begin
			// Timer ticks are counted in phi0, which is 5 MHz.
			//
			// The datasheet's table is internally inconsistent about what fCK
			// means, and two SDK comments pin the real periods:
			//
			//   TM1C=80h (sel 000) TM1D=D0h -> 41.6 us  = 208 phi0
			//   TM1C=81h (sel 001) TM1D=18h -> 4.9152 ms = 24576 phi0
			//
			// Both hold only if the interrupt is taken on every compare, with
			// selector 000 counting single phi0 ticks and 001 counting 1024 of
			// them. So row 000 reads as literal fCK while rows 001-111 read
			// against the prescaler input, fCK/2.
			//
			// Do not fold a factor of two into the interrupt path instead. The
			// timer output really does invert on every compare, but selector 000
			// already ticks once per phi0 and cannot be halved to pay for it, so
			// that model breaks the 41.6 us speech cadence. Tried and reverted;
			// see decision 0006.
			case (sel)
				3'b000: timer_prescale_reload = 16'd0;
				3'b001: timer_prescale_reload = 16'd1023;
				3'b010: timer_prescale_reload = 16'd2047;
				3'b011: timer_prescale_reload = 16'd4095;
				3'b100: timer_prescale_reload = 16'd8191;
				3'b101: timer_prescale_reload = 16'd16383;
				3'b110: timer_prescale_reload = 16'd32767;
				default: timer_prescale_reload = 16'd65535;
			endcase
		end
	endfunction

	task apply_reset_state;
		begin
			tm0d_q <= 8'h00;
			tm0_div_q <= 16'd0;
			tm0_out_q <= 1'b0;
			tm1d_q <= 8'h00;
			tm1_div_q <= 16'd0;
			tm1_out_q <= 1'b0;
			clkt_count_q <= 6'h00;
			clkt_div_q <= 24'h000000;
			tm0c_write_seq_prev_q <= 2'b00;
			tm0_reload_write_seq_prev_q <= 2'b00;
			tm1c_write_seq_prev_q <= 2'b00;
			tm1_reload_write_seq_prev_q <= 2'b00;
			clkt_write_seq_prev_q <= 2'b00;
			clkt_run_prev_q <= 1'b0;
			clkt_minute_prev_q <= 1'b0;
		end
	endtask

	always @(*) begin : timer_event_logic
		reg [7:0] tm0d_v;
		reg [15:0] tm0_div_v;
		reg [7:0] tm1d_v;
		reg [15:0] tm1_div_v;
		reg [5:0] clkt_count_v;
		reg [23:0] clkt_div_v;
		tm0_irq_event_v = 1'b0;
		tm1_irq_event_v = 1'b0;
		clkt_second_event_v = 1'b0;
		clk_irq_event_v = 1'b0;

		tm0d_v = tm0d_q;
		tm0_div_v = tm0_div_q;
		if (tm0c_write_seq_i != tm0c_write_seq_prev_q) begin
			// GUESS: the datasheet defines the control fields but does not
			// state write-restart timing. MAME treats every TM0C write as a
			// restart edge: bit 7 becomes enable, bits 2:0 select the
			// prescaler, and the visible TM0D counter is cleared.
			tm0d_v = 8'h00;
			tm0_div_v = timer_prescale_reload(tm0c_i[2:0]);
		end else if (tm0_reload_write_seq_i != tm0_reload_write_seq_prev_q) begin
			// TM0D/TM1D are time constants on write but counter contents on read.
			// GUESS: MAME also clears the visible counter on a time-constant
			// write; the datasheet does not state whether a live counter resets.
			tm0d_v = 8'h00;
		end
		if (!main_stopped_i && tm0c_i[7] && (tm0_div_v == 16'h0000)) begin
			// The datasheet states that compare coincidence inverts the timer
			// output. GUESS: the counter also rearms to zero on that compare,
			// matching MAME's periodic upcounter model.
			if ((tm0_reload_i == 8'h00) || ((tm0d_v + 8'h01) >= tm0_reload_i)) begin
				tm0_irq_event_v = 1'b1;
			end
		end

		tm1d_v = tm1d_q;
		tm1_div_v = tm1_div_q;
		if (tm1c_write_seq_i != tm1c_write_seq_prev_q) begin
			// GUESS: keep TM1C write-restart behavior paired with TM0C. Raw
			// storage is outside this block, while active hardware uses only
			// bit 7 and bits 2:0.
			tm1d_v = 8'h00;
			tm1_div_v = timer_prescale_reload(tm1c_i[2:0]);
		end else if (tm1_reload_write_seq_i != tm1_reload_write_seq_prev_q) begin
			// GUESS: mirror TM0D/MAME live-write counter clear behavior.
			tm1d_v = 8'h00;
		end
		if (!main_stopped_i && tm1c_i[7] && (tm1_div_v == 16'h0000)) begin
			if ((tm1_reload_i == 8'h00) || ((tm1d_v + 8'h01) >= tm1_reload_i)) begin
				tm1_irq_event_v = 1'b1;
			end
		end

		clkt_count_v = clkt_count_q;
		clkt_div_v = clkt_div_q;
		if ((clkt_write_seq_i != clkt_write_seq_prev_q) ||
			(clkt_run_i != clkt_run_prev_q) ||
			(clkt_minute_i != clkt_minute_prev_q)) begin
			if (!clkt_run_i) begin
				clkt_count_v = 6'h00;
				clkt_div_v = 24'h000000;
			end else begin
				// GUESS: the datasheet states reset behavior when run/reset is
				// zero, but not whether a write of run=1 restarts the divider.
				// Re-arm on CLKT writes and mode changes to match MAME.
				clkt_div_v = CLKT_1S_PHI0_CYCLES_W - 24'd1;
			end
		end
		if (clkt_run_i && (clkt_div_v == 24'h000000)) begin
			// GUESS: internal one-second bookkeeping pulse for the wrapper RTC.
			// The public CLK interrupt remains selected by CLKT[6] below.
			clkt_second_event_v = 1'b1;
			// Second mode interrupts on every tick, all sixty of them. Minute
			// mode interrupts only as the seconds counter wraps 59 -> 0.
			if (!clkt_minute_i || (clkt_count_v == 6'd59)) begin
				clk_irq_event_v = 1'b1;
			end
		end
	end

	always @(posedge clk_sys_i) begin
		if (!resetb_i) begin
			apply_reset_state();
		end else if (core_reset_i) begin
			apply_reset_state();
		end else if (ss_wren_i) begin
			case (ss_addr_i)
				5'd0: tm0d_q <= ss_wdata_i;
				5'd1: tm0_div_q[7:0] <= ss_wdata_i;
				5'd2: tm0_div_q[15:8] <= ss_wdata_i;
				5'd3: tm0_out_q <= ss_wdata_i[0];
				5'd4: tm1d_q <= ss_wdata_i;
				5'd5: tm1_div_q[7:0] <= ss_wdata_i;
				5'd6: tm1_div_q[15:8] <= ss_wdata_i;
				5'd7: tm1_out_q <= ss_wdata_i[0];
				5'd8: clkt_count_q <= ss_wdata_i[5:0];
				5'd9: clkt_div_q[7:0] <= ss_wdata_i;
				5'd10: clkt_div_q[15:8] <= ss_wdata_i;
				5'd11: clkt_div_q[23:16] <= ss_wdata_i;
				5'd12: begin
					tm0c_write_seq_prev_q <= ss_wdata_i[1:0];
					tm0_reload_write_seq_prev_q <= ss_wdata_i[3:2];
					tm1c_write_seq_prev_q <= ss_wdata_i[5:4];
					tm1_reload_write_seq_prev_q <= ss_wdata_i[7:6];
				end
				5'd13: begin
					clkt_write_seq_prev_q <= ss_wdata_i[1:0];
					clkt_run_prev_q <= ss_wdata_i[2];
					clkt_minute_prev_q <= ss_wdata_i[3];
				end
				default: begin
				end
			endcase
		end else if (phi0_ce_i) begin : timer_seq_logic
			reg [7:0] tm0d_v;
			reg [15:0] tm0_div_v;
			reg tm0_out_v;
			reg [7:0] tm1d_v;
			reg [15:0] tm1_div_v;
			reg tm1_out_v;
			reg [5:0] clkt_count_v;
			reg [23:0] clkt_div_v;

			tm0d_v = tm0d_q;
			tm0_div_v = tm0_div_q;
			tm0_out_v = tm0_out_q;
			if (tm0c_write_seq_i != tm0c_write_seq_prev_q) begin
				// See the matching combinational event path above for
				// the guessed MAME write-restart behavior being modeled here.
				tm0d_v = 8'h00;
				tm0_div_v = timer_prescale_reload(tm0c_i[2:0]);
				// GUESS: the datasheet does not state the output phase after a
				// control write; restart low for deterministic UART/P37 phase.
				tm0_out_v = 1'b0;
			end else if (tm0_reload_write_seq_i != tm0_reload_write_seq_prev_q) begin
				// GUESS: MAME clears the visible counter on a live TM0D write.
				tm0d_v = 8'h00;
			end
			if (!main_stopped_i && tm0c_i[7]) begin
				if (tm0_div_v != 16'h0000) begin
					tm0_div_v = tm0_div_v - 16'h0001;
				end else begin
					tm0_div_v = timer_prescale_reload(tm0c_i[2:0]);
					if ((tm0_reload_i == 8'h00) || ((tm0d_v + 8'h01) >= tm0_reload_i)) begin
						// GUESS: compare equality also clears the visible
						// upcounter so the time constant is a repeat period.
						tm0d_v = 8'h00;
						tm0_out_v = ~tm0_out_v;
					end else begin
						tm0d_v = tm0d_v + 8'h01;
					end
				end
			end

			tm1d_v = tm1d_q;
			tm1_div_v = tm1_div_q;
			tm1_out_v = tm1_out_q;
			if (tm1c_write_seq_i != tm1c_write_seq_prev_q) begin
				// See TM0C: every control write is treated as a guessed
				// restart edge.
				tm1d_v = 8'h00;
				tm1_div_v = timer_prescale_reload(tm1c_i[2:0]);
				tm1_out_v = 1'b0;
			end else if (tm1_reload_write_seq_i != tm1_reload_write_seq_prev_q) begin
				// GUESS: mirror TM0D/MAME live-write counter clear behavior.
				tm1d_v = 8'h00;
			end
			if (!main_stopped_i && tm1c_i[7]) begin
				if (tm1_div_v != 16'h0000) begin
					tm1_div_v = tm1_div_v - 16'h0001;
				end else begin
					tm1_div_v = timer_prescale_reload(tm1c_i[2:0]);
					if ((tm1_reload_i == 8'h00) || ((tm1d_v + 8'h01) >= tm1_reload_i)) begin
						// GUESS: match TM0/MAME periodic rearm behavior.
						tm1d_v = 8'h00;
						tm1_out_v = ~tm1_out_v;
					end else begin
						tm1d_v = tm1d_v + 8'h01;
					end
				end
			end

			clkt_count_v = clkt_count_q;
			clkt_div_v = clkt_div_q;
			if ((clkt_write_seq_i != clkt_write_seq_prev_q) ||
				(clkt_run_i != clkt_run_prev_q) ||
				(clkt_minute_i != clkt_minute_prev_q)) begin
				if (!clkt_run_i) begin
					clkt_count_v = 6'h00;
					clkt_div_v = 24'h000000;
				end else begin
					// GUESS: match the event path's CLKT write/mode-change rearm.
					clkt_div_v = CLKT_1S_PHI0_CYCLES_W - 24'd1;
				end
			end
			if (clkt_run_i) begin
				if (clkt_div_v != 24'h000000) begin
					clkt_div_v = clkt_div_v - 24'h000001;
				end else begin
					clkt_div_v = CLKT_1S_PHI0_CYCLES_W - 24'd1;
					if (clkt_count_v == 6'd59) begin
						clkt_count_v = 6'd0;
					end else begin
						clkt_count_v = clkt_count_v + 6'd1;
					end
				end
			end

			tm0d_q <= tm0d_v;
			tm0_div_q <= tm0_div_v;
			tm0_out_q <= tm0_out_v;
			tm1d_q <= tm1d_v;
			tm1_div_q <= tm1_div_v;
			tm1_out_q <= tm1_out_v;
			clkt_count_q <= clkt_count_v;
			clkt_div_q <= clkt_div_v;
			tm0c_write_seq_prev_q <= tm0c_write_seq_i;
			tm0_reload_write_seq_prev_q <= tm0_reload_write_seq_i;
			tm1c_write_seq_prev_q <= tm1c_write_seq_i;
			tm1_reload_write_seq_prev_q <= tm1_reload_write_seq_i;
			clkt_write_seq_prev_q <= clkt_write_seq_i;
			clkt_run_prev_q <= clkt_run_i;
			clkt_minute_prev_q <= clkt_minute_i;
		end
	end
endmodule
