// Copyright (c) 2026 Jamie Blanks

module gamecom_audio_output
(
	input wire clk_sys_i,
	input wire ce_i,
	input wire reset_i,
	input wire [7:0] sample_i,

	output wire [15:0] sample_o
);

	// Fixed-point analog approximation at the native sound clock. The input is
	// the raw unsigned DAC voltage domain, where code 00H is the reset/off level
	// and 80H is the midpoint of an active unsigned PCM stream. The first pole
	// models DAC/output reconstruction; the second is a slow DC servo that lets
	// a held DAC code decay like the AC-coupled real output stage.
	localparam integer INPUT_FRAC_BITS = 20;
	localparam integer OUTPUT_SHIFT = INPUT_FRAC_BITS - 8;
	localparam integer RECONSTRUCTION_SHIFT = 6;
	localparam integer DC_SERVO_SHIFT = 15;

	wire signed [31:0] input_fixed_w =
		{4'b0000, sample_i, {INPUT_FRAC_BITS{1'b0}}};

	reg signed [31:0] reconstruction_q;
	reg signed [31:0] dc_servo_q;

	wire signed [31:0] reconstruction_error_w = input_fixed_w - reconstruction_q;
	wire signed [31:0] dc_servo_error_w = reconstruction_q - dc_servo_q;
	wire signed [31:0] ac_output_w = reconstruction_q - dc_servo_q;
	wire signed [31:0] pcm_output_w = ac_output_w >>> OUTPUT_SHIFT;

	assign sample_o =
		(pcm_output_w > 32'sd32767) ? 16'hFFFF :
		(pcm_output_w < -32'sd32768) ? 16'h0000 :
		(pcm_output_w[15:0] + 16'h8000);

	always @(posedge clk_sys_i) begin
		if (reset_i) begin
			reconstruction_q <= 32'sd0;
			dc_servo_q <= 32'sd0;
		end else if (ce_i) begin
			reconstruction_q <= reconstruction_q +
				(reconstruction_error_w >>> RECONSTRUCTION_SHIFT);
			dc_servo_q <= dc_servo_q +
				(dc_servo_error_w >>> DC_SERVO_SHIFT);
		end
	end

endmodule
