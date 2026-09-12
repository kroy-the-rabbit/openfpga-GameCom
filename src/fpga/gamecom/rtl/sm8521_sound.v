// Copyright (c) 2026 Jamie Blanks

module sm8521_sound (
	input wire clk_sys_i,
	input wire resetb_i,
	input wire core_reset_i,
	input wire fck_ce_i,
	input wire stopped_i,
	input wire [7:0] sgc_i,
	input wire [7:0] sg0l_i,
	input wire [7:0] sg1l_i,
	input wire [7:0] sg2l_i,
	input wire [15:0] sg0t_i,
	input wire [15:0] sg1t_i,
	input wire [15:0] sg2t_i,
	input wire [7:0] sgda_i,
	input wire sgda_write_strobe_i,
	input wire [127:0] sg0_wave_i,
	input wire [127:0] sg1_wave_i,
	output reg [7:0] sound_pcm_o,
	input wire [4:0] ss_addr_i,
	input wire ss_wren_i,
	input wire [7:0] ss_wdata_i,
	output reg [7:0] ss_rdata_o
);
	wire [11:0] sg0_period_w = sg0t_i[11:0];
	wire [11:0] sg1_period_w = sg1t_i[11:0];
	wire [11:0] sg2_period_w = sg2t_i[11:0];
	// Real SG2 captures with SG2TL=FFH show the 15-bit noise pattern repeating
	// after 32767 * 256 5 MHz ticks, so the divider interval is N+1 ticks.
	// GUESS: applying that same divider interval to SG0/SG1 follows the datasheet
	// grouping of all SGxT registers, but has not been separately tone-captured.
	wire [15:0] sg0_reload_w = {4'h0, sg0_period_w};
	wire [15:0] sg1_reload_w = {4'h0, sg1_period_w};
	wire [15:0] sg2_reload_w = {4'h0, sg2_period_w};
	// Real hardware direct-DAC mode is exclusive with SG0/SG1/SG2 output.
	wire direct_dac_enabled_w = sgc_i[7] && sgc_i[3] && (sgc_i[2:0] == 3'b000);
	// SGxTH is RAM-backed for readback, but only SGxTH[3:0]:SGxTL is part of
	// the hardware time constant. Keep the sound core's active period 12-bit so
	// retained register high bits cannot leak into the active divider.
	wire sg0_period_valid_w = sg0_period_w > 12'd1;
	wire sg1_period_valid_w = sg1_period_w > 12'd1;
	wire sg2_period_valid_w = sg2_period_w > 12'd1;
	wire sg0_active_w = sgc_i[7] && sgc_i[0] && sg0_period_valid_w;
	wire sg1_active_w = sgc_i[7] && sgc_i[1] && sg1_period_valid_w;
	wire sg2_active_w = sgc_i[7] && sgc_i[2] && sg2_period_valid_w;

	reg sg0_run_q;
	reg sg1_run_q;
	reg sg2_run_q;
	reg [4:0] sg0_index_q;
	reg [4:0] sg1_index_q;
	reg [15:0] sg0_div_q;
	reg [15:0] sg1_div_q;
	reg [15:0] sg2_div_q;
	reg signed [4:0] sg0_sample_q;
	reg signed [4:0] sg1_sample_q;
	reg signed [4:0] sg2_sample_q;
	reg [14:0] sg2_lfsr_q;
	reg [7:0] direct_dac_q;

	wire [7:0] direct_dac_live_w = sgda_write_strobe_i ? sgda_i : direct_dac_q;
	wire signed [8:0] direct_dac_delta_w = {1'b0, direct_dac_live_w} - 9'sd128;
	// SGC channel bits are output enables. Disabled generators do not advance
	// or contribute to the mixer, preserving phase/state across SGC off/on.
	wire sg0_mix_enabled_w = sgc_i[7] && sgc_i[0];
	wire sg1_mix_enabled_w = sgc_i[7] && sgc_i[1];
	wire sg2_mix_enabled_w = sgc_i[7] && sgc_i[2];

	reg signed [10:0] mix_comb_v;
	reg [7:0] dac_comb_v;

	task apply_reset_state;
		begin
			sg0_run_q <= 1'b0;
			sg1_run_q <= 1'b0;
			sg2_run_q <= 1'b0;
			sg0_index_q <= 5'd0;
			sg1_index_q <= 5'd0;
			sg0_div_q <= 16'h0000;
			sg1_div_q <= 16'h0000;
			sg2_div_q <= 16'h0000;
			sg0_sample_q <= 5'sd0;
			sg1_sample_q <= 5'sd0;
			sg2_sample_q <= 5'sd0;
			sg2_lfsr_q <= 15'h7FFF;
			direct_dac_q <= 8'h00;
			sound_pcm_o <= 8'h00;
		end
	endtask

	function signed [4:0] wave_sample;
		input [127:0] wave;
		input [4:0] step;
		reg [3:0] nib_v;
		begin
			// SGxW is 32 signed 4-bit samples, low nibble of each byte first,
			// so sample n starts at bit 4n.
			nib_v = wave[{step, 2'b00} +: 4];
			wave_sample = {nib_v[3], nib_v};
		end
	endfunction


	function signed [10:0] scale_wave_sample;
		input signed [4:0] sample;
		input [4:0] level;
		reg signed [10:0] base_v;
		reg signed [15:0] acc_v;
		reg signed [15:0] shift_v;
		reg signed [10:0] scaled_v;
		integer bit_i;
		begin
			// GUESS: the datasheet says 4-bit channel data is expanded 16x and
			// attenuated by SGxL/32. The fixed-point mapping below converts that
			// analog/DAC-domain wording to PCM.
			base_v = {{2{sample[4]}}, sample, 4'b0000};
			acc_v = 16'sd0;
			for (bit_i = 0; bit_i < 5; bit_i = bit_i + 1) begin
				shift_v = {{5{base_v[10]}}, base_v};
				if (level[bit_i]) acc_v = acc_v + (shift_v <<< bit_i);
			end
			scaled_v = {acc_v[15], acc_v[14:5]};
			scale_wave_sample = scaled_v;
		end
	endfunction

	// SG2 uses the same full-width LFSR topology as the Game Boy noise
	// channel: bit0 xor bit1 feeds bit14 while the state shifts right. An
	// all-zero state would never leave itself, so it reloads to all ones.
	wire [14:0] sg2_lfsr_shift_w = {sg2_lfsr_q[0] ^ sg2_lfsr_q[1], sg2_lfsr_q[14:1]};
	wire [14:0] sg2_lfsr_next_w =
		(sg2_lfsr_shift_w == 15'h0000) ? 15'h7FFF : sg2_lfsr_shift_w;

	always @(*) begin
		mix_comb_v = 11'sd128;
		if (sg0_mix_enabled_w) mix_comb_v = mix_comb_v + scale_wave_sample(sg0_sample_q, sg0l_i[4:0]);
		if (sg1_mix_enabled_w) mix_comb_v = mix_comb_v + scale_wave_sample(sg1_sample_q, sg1l_i[4:0]);
		if (sg2_mix_enabled_w) mix_comb_v = mix_comb_v + scale_wave_sample(sg2_sample_q, sg2l_i[4:0]);
		if (direct_dac_enabled_w) begin
			mix_comb_v = mix_comb_v + {{2{direct_dac_delta_w[8]}}, direct_dac_delta_w};
		end

		if (!sgc_i[7]) begin
			// The physical DAC is unsigned and returns to code zero when the
			// global sound output is disabled. Active PCM midpoint remains 80H.
			dac_comb_v = 8'h00;
		end else if (mix_comb_v < 11'sd0) begin
			dac_comb_v = 8'h00;
		end else if (mix_comb_v > 11'sd255) begin
			dac_comb_v = 8'hFF;
		end else begin
			dac_comb_v = mix_comb_v[7:0];
		end
	end

	always @(*) begin
		case (ss_addr_i)
			5'd0: ss_rdata_o = {5'b00000, sg2_run_q, sg1_run_q, sg0_run_q};
			5'd1: ss_rdata_o = {3'b000, sg0_index_q};
			5'd2: ss_rdata_o = sg0_div_q[7:0];
			5'd3: ss_rdata_o = sg0_div_q[15:8];
			5'd4: ss_rdata_o = sg1_div_q[7:0];
			5'd5: ss_rdata_o = sg1_div_q[15:8];
			5'd6: ss_rdata_o = sg2_div_q[7:0];
			5'd7: ss_rdata_o = sg2_div_q[15:8];
			5'd8: ss_rdata_o = {3'b000, sg0_sample_q};
			5'd9: ss_rdata_o = {3'b000, sg1_sample_q};
			5'd10: ss_rdata_o = {3'b000, sg2_sample_q};
			5'd11: ss_rdata_o = sg2_lfsr_q[7:0];
			5'd12: ss_rdata_o = {1'b0, sg2_lfsr_q[14:8]};
			5'd13: ss_rdata_o = 8'h00;
			5'd14: ss_rdata_o = direct_dac_q;
			5'd15: ss_rdata_o = sound_pcm_o;
			default: ss_rdata_o = 8'h00;
		endcase
	end

	always @(posedge clk_sys_i) begin
		if (!resetb_i) begin
			apply_reset_state();
		end else if (core_reset_i) begin
			apply_reset_state();
		end else if (ss_wren_i) begin
			case (ss_addr_i)
				5'd0: begin
					sg0_run_q <= ss_wdata_i[0];
					sg1_run_q <= ss_wdata_i[1];
					sg2_run_q <= ss_wdata_i[2];
				end
				5'd1: sg0_index_q <= ss_wdata_i[4:0];
				5'd2: sg0_div_q[7:0] <= ss_wdata_i;
				5'd3: sg0_div_q[15:8] <= ss_wdata_i;
				5'd4: sg1_div_q[7:0] <= ss_wdata_i;
				5'd5: sg1_div_q[15:8] <= ss_wdata_i;
				5'd6: sg2_div_q[7:0] <= ss_wdata_i;
				5'd7: sg2_div_q[15:8] <= ss_wdata_i;
				5'd8: sg0_sample_q <= ss_wdata_i[4:0];
				5'd9: sg1_sample_q <= ss_wdata_i[4:0];
				5'd10: sg2_sample_q <= ss_wdata_i[4:0];
				5'd11: sg2_lfsr_q[7:0] <= ss_wdata_i;
				5'd12: sg2_lfsr_q[14:8] <= ss_wdata_i[6:0];
				5'd13: begin
				end
				5'd14: direct_dac_q <= ss_wdata_i;
				5'd15: sound_pcm_o <= ss_wdata_i;
				default: begin
				end
			endcase
		end else begin
			if (sgda_write_strobe_i) begin
				direct_dac_q <= sgda_i;
			end

			if (fck_ce_i) begin
				// SG0/1/2 advance on the sound FCK tick from the parent, but
				// STOP mode halts the waveform generator until the CPU wakes.
				if (stopped_i) begin
					sound_pcm_o <= sgc_i[7] ? 8'h80 : 8'h00;
				end else begin
					if (sg0_active_w && !sg0_run_q) begin
						sg0_run_q <= 1'b1;
						sg0_index_q <= 5'd1;
						sg0_div_q <= sg0_reload_w;
						// The SM8521 datasheet names step 0 as the first waveform
						// value, and related Sharp waveform-generator documentation
						// says generation always starts from step 0. Drive it on the
						// start edge instead of inserting an undocumented mute step.
						sg0_sample_q <= wave_sample(sg0_wave_i, 5'd0);
					end else if (sg0_active_w) begin
						// GUESS: follow Furnace's SGC lifetime model. Disabling output
						// pauses the generator instead of resetting phase.
						if (sg0_div_q != 16'h0000) begin
							sg0_div_q <= sg0_div_q - 16'd1;
						end else begin
							sg0_div_q <= (sg0_period_w == 12'd0) ? 16'd0 : sg0_reload_w;
							sg0_sample_q <= wave_sample(sg0_wave_i, sg0_index_q);
							if (sg0_index_q == 5'd31) begin
								sg0_index_q <= 5'd0;
							end else begin
								sg0_index_q <= sg0_index_q + 5'd1;
							end
						end
					end else if (!sg0_period_valid_w) begin
						sg0_run_q <= 1'b0;
						sg0_index_q <= 5'd0;
						sg0_div_q <= 16'h0000;
						sg0_sample_q <= 5'sd0;
					end

					if (sg1_active_w && !sg1_run_q) begin
						sg1_run_q <= 1'b1;
						sg1_index_q <= 5'd1;
						sg1_div_q <= sg1_reload_w;
						// Match SG0: start with waveform step 0 already selected.
						sg1_sample_q <= wave_sample(sg1_wave_i, 5'd0);
					end else if (sg1_active_w) begin
						// GUESS: same no-live-phase-reset behavior as SG0.
						if (sg1_div_q != 16'h0000) begin
							sg1_div_q <= sg1_div_q - 16'd1;
						end else begin
							sg1_div_q <= (sg1_period_w == 12'd0) ? 16'd0 : sg1_reload_w;
							sg1_sample_q <= wave_sample(sg1_wave_i, sg1_index_q);
							if (sg1_index_q == 5'd31) begin
								sg1_index_q <= 5'd0;
							end else begin
								sg1_index_q <= sg1_index_q + 5'd1;
							end
						end
					end else if (!sg1_period_valid_w) begin
						sg1_run_q <= 1'b0;
						sg1_index_q <= 5'd0;
						sg1_div_q <= 16'h0000;
						sg1_sample_q <= 5'sd0;
					end

					if (sg2_active_w && !sg2_run_q) begin
						sg2_run_q <= 1'b1;
						sg2_div_q <= sg2_reload_w;
						sg2_lfsr_q <= 15'h7FFF;
						sg2_sample_q <= 5'sd0;
					end else if (sg2_active_w) begin
						if (sg2_div_q != 16'h0000) begin
							sg2_div_q <= sg2_div_q - 16'd1;
						end else begin
							sg2_div_q <= (sg2_period_w == 12'd0) ? 16'd0 : sg2_reload_w;
							sg2_lfsr_q <= sg2_lfsr_next_w;
							// GUESS: use the same signed 4-bit full-scale domain as SG0/SG1.
							if (!sg2_lfsr_next_w[0]) begin
								sg2_sample_q <= 5'sd7;
							end else begin
								sg2_sample_q <= -5'sd8;
							end
						end
					end else if (!sg2_period_valid_w) begin
						sg2_run_q <= 1'b0;
						sg2_div_q <= 16'h0000;
						sg2_sample_q <= 5'sd0;
						sg2_lfsr_q <= 15'h7FFF;
					end

					// Real hardware makes direct DAC exclusive with SG channels.
					// Saturation models the unspecified D/A over-range behavior.
					sound_pcm_o <= dac_comb_v;
				end
			end else if (sgda_write_strobe_i && direct_dac_enabled_w && !stopped_i) begin
				// GUESS: make direct-DAC writes audible immediately between SG
				// ticks in pure direct mode. This matches speech needs, but the
				// datasheet only states SGDA directly transfers to the mixer.
				sound_pcm_o <= sgda_i;
			end
		end
	end
endmodule
