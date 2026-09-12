// Copyright (c) 2026 Jamie Blanks

//============================================================================
// Game.com MiSTer byte-wide cheat engine
//
// Standard MiSTer record layout after little-endian IOCTL byte collection:
// 128       record-valid strobe
// 105:104   replacement method
// 102:100   value width
// 96        compare enable
// 95:64     address (the Game.com uses the low 16 bits)
// 63:32     compare value
// 31:0      replacement value
// Remaining flag and address bits are reserved by the common record format.
//
// Based on the MiSTer cheat-engine convention originally implemented by
// Kitrinx and used by other console and arcade cores.
//============================================================================

module gamecom_cheat_engine
#(
	parameter integer MAX_CODES = 12
)
(
	input wire clk_sys_i,
	input wire clear_i,
	input wire enable_i,
	input wire [128:0] code_i,
	input wire [15:0] addr_i,
	input wire [7:0] data_i,
	output reg [7:0] data_o,
	output wire available_o
);

	localparam integer COUNT_WIDTH = $clog2(MAX_CODES + 1);
	localparam [COUNT_WIDTH-1:0] MAX_CODES_COUNT = MAX_CODES[COUNT_WIDTH-1:0];
	localparam [1:0] METHOD_OR = 2'd1;
	localparam [1:0] METHOD_AND = 2'd2;

	wire code_valid_w = code_i[128];
	wire [1:0] code_method_w = code_i[105:104];
	wire [2:0] code_width_w = code_i[102:100];
	wire code_compare_enable_w = code_i[96];
	wire [15:0] code_addr_w = code_i[79:64];
	wire [31:0] code_compare_w = code_i[63:32];
	wire [31:0] code_value_w = code_i[31:0];

	reg [15:0] code_addr_q [0:MAX_CODES-1];
	reg [7:0] code_compare_q [0:MAX_CODES-1];
	reg code_compare_enable_q [0:MAX_CODES-1];

	// All three methods are (data & and_mask) | or_mask, so the masks are built
	// once at load time and the method mux disappears from the read path:
	//   replace  00 / value      or  FF / value      and  value / 00
	reg [7:0] code_and_q [0:MAX_CODES-1];
	reg [7:0] code_or_q [0:MAX_CODES-1];

	reg [COUNT_WIDTH-1:0] code_count_q = {COUNT_WIDTH{1'b0}};
	reg [2:0] pending_count_q = 3'd0;
	reg [15:0] pending_addr_q = 16'h0000;
	reg [31:0] pending_value_q = 32'h00000000;
	reg [31:0] pending_compare_q = 32'h00000000;
	reg pending_compare_enable_q = 1'b0;
	reg [1:0] pending_method_q = 2'b00;

	// Address matching cannot depend on the byte being read, so it is resolved
	// one clock ahead and kept out of the path from data_i to data_o. The CPU
	// holds the access address for at least a full phi0 tick before it samples
	// the byte, so this is settled well before it is used.
	reg [MAX_CODES-1:0] addr_hit_q;

	wire [MAX_CODES-1:0] match_w;      // address hit and the compare byte agrees
	wire [MAX_CODES-1:0] outranked_w;  // a later code also matches
	wire [MAX_CODES-1:0] select_w;     // exactly one, the last matching code
	wire no_match_w = ~|match_w;
	reg [7:0] final_and_v;
	reg [7:0] final_or_v;
	integer code_idx;

	assign available_o = code_count_q != {COUNT_WIDTH{1'b0}};

	genvar gi;
	generate
		for (gi = 0; gi < MAX_CODES; gi = gi + 1) begin : g_match
			assign match_w[gi] = enable_i && addr_hit_q[gi] &&
				(!code_compare_enable_q[gi] || (code_compare_q[gi] == data_i));

			// Written as a reduction rather than a chain of if statements so
			// this stays a balanced tree instead of a twelve-deep ripple.
			if (gi == (MAX_CODES - 1)) begin : g_last
				assign outranked_w[gi] = 1'b0;
			end else begin : g_rest
				assign outranked_w[gi] = |match_w[MAX_CODES-1:gi+1];
			end
		end
	endgenerate

	assign select_w = match_w & ~outranked_w;

	// select_w is one-hot, so a masked OR picks the winner in a balanced tree.
	// With no match the pass-through masks join the same tree, which keeps the
	// read path at one AND and one OR after it.
	always @* begin
		final_and_v = {8{no_match_w}};
		final_or_v = 8'h00;
		for (code_idx = 0; code_idx < MAX_CODES; code_idx = code_idx + 1) begin
			final_and_v = final_and_v |
				({8{select_w[code_idx]}} & code_and_q[code_idx]);
			final_or_v = final_or_v |
				({8{select_w[code_idx]}} & code_or_q[code_idx]);
		end
	end

	always @* begin
		data_o = (data_i & final_and_v) | final_or_v;
	end

	always @(posedge clk_sys_i) begin
		for (code_idx = 0; code_idx < MAX_CODES; code_idx = code_idx + 1) begin
			addr_hit_q[code_idx] <=
				(code_idx < code_count_q) && (code_addr_q[code_idx] == addr_i);
		end
	end

	always @(posedge clk_sys_i) begin
		if (clear_i) begin
			code_count_q <= {COUNT_WIDTH{1'b0}};
			pending_count_q <= 3'd0;
		end else begin
			if ((pending_count_q != 3'd0) && (code_count_q < MAX_CODES_COUNT)) begin
				code_addr_q[code_count_q] <= pending_addr_q;
				code_compare_q[code_count_q] <= pending_compare_q[7:0];
				code_compare_enable_q[code_count_q] <= pending_compare_enable_q;
				code_and_q[code_count_q] <=
					(pending_method_q == METHOD_OR) ? 8'hFF :
					(pending_method_q == METHOD_AND) ? pending_value_q[7:0] : 8'h00;
				code_or_q[code_count_q] <=
					(pending_method_q == METHOD_AND) ? 8'h00 : pending_value_q[7:0];
				code_count_q <= code_count_q + {{(COUNT_WIDTH-1){1'b0}}, 1'b1};
				pending_addr_q <= pending_addr_q + 16'h0001;
				pending_value_q <= {8'h00, pending_value_q[31:8]};
				pending_compare_q <= {8'h00, pending_compare_q[31:8]};
				pending_count_q <= pending_count_q - 3'd1;
			end else if (pending_count_q != 3'd0) begin
				pending_count_q <= 3'd0;
			end

			if (code_valid_w && (pending_count_q == 3'd0)) begin
				pending_addr_q <= code_addr_w;
				pending_value_q <= code_value_w;
				pending_compare_q <= code_compare_w;
				pending_compare_enable_q <= code_compare_enable_w;
				pending_method_q <= code_method_w;
				case (code_width_w)
					3'b000, 3'b001: pending_count_q <= 3'd1;
					3'b010: pending_count_q <= code_i[64] ? 3'd0 : 3'd2;
					3'b100: pending_count_q <= |code_i[65:64] ? 3'd0 : 3'd4;
					default: pending_count_q <= 3'd0;
				endcase
			end
		end
	end

endmodule
