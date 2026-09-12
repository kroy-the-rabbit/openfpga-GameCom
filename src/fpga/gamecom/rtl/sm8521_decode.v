// Copyright (c) 2026 Jamie Blanks

module sm8521_decode
(
	input wire [7:0] opcode_i,
	output reg [3:0] action_o,
	output reg [5:0] op_class_o,
	output reg [2:0] op_stage_o,
	output reg [5:0] next_state_o,
	output reg target_reg_we_o,
	output reg [3:0] target_reg_o,
	output reg pair_base_we_o,
	output reg [3:0] pair_base_o,
	output reg target_addr_we_o,
	output reg [7:0] target_addr_o
);

	localparam ACT_NORMAL = 4'd0;
	localparam ACT_FETCH  = 4'd1;
	localparam ACT_ILL    = 4'd2;
	localparam ACT_HALT   = 4'd3;
	localparam ACT_STOP   = 4'd4;
	localparam ACT_RET    = 4'd5;
	localparam ACT_IRET   = 4'd6;
	localparam ACT_CLRC   = 4'd7;
	localparam ACT_COMC   = 4'd8;
	localparam ACT_SETC   = 4'd9;
	localparam ACT_EI     = 4'd10;
	localparam ACT_DI     = 4'd11;

	localparam ST_FETCH_SETUP       = 6'd1;
	localparam ST_OP_READ8_SETUP    = 6'd4;
	localparam ST_OP_READ16H_SETUP  = 6'd6;
	localparam ST_OP_READ8_16H_SETUP = 6'd28;

	localparam CLASS_NONE             = 6'd0;
	localparam CLASS_MOVI_REG         = 6'd1;
	localparam CLASS_MOVI_SFR         = 6'd2;
	localparam CLASS_MOV_REG_TO_FIXED = 6'd3;
	localparam CLASS_MOV_FIXED_TO_REG = 6'd4;
	localparam CLASS_MOVW_IMM         = 6'd5;
	localparam CLASS_BR               = 6'd6;
	localparam CLASS_JMP              = 6'd7;
	localparam CLASS_CLR_FIXED        = 6'd8;
	localparam CLASS_MOVPS0_IMM       = 6'd9;
	localparam CLASS_MOVW_FIXED       = 6'd11;
	localparam CLASS_MEM_MOV8         = 6'd12;
	localparam CLASS_MEM_CMP8         = 6'd13;
	localparam CLASS_MEM_BYTE_ALU     = 6'd14;
	localparam CLASS_DBNZ             = 6'd15;
	localparam CLASS_MEM_STORE8       = 6'd16;
	localparam CLASS_FIXED_BYTE_OP    = 6'd17;
	localparam CLASS_CALL_ABS         = 6'd18;
	localparam CLASS_MEM_MOVW_LOAD    = 6'd19;
	localparam CLASS_MEM_MOVW_STORE   = 6'd20;
	localparam CLASS_MOVW_COMPACT     = 6'd21;
	localparam CLASS_JMP_INDIRECT     = 6'd22;
	localparam CLASS_CALL_INDIRECT    = 6'd23;
	localparam CLASS_MOVW_DIRECT      = 6'd24;
	localparam CLASS_DIRECT_IMM_OP    = 6'd25;
	localparam CLASS_WORD_RR_OP       = 6'd26;
	localparam CLASS_WORD_IMM_OP      = 6'd27;
	localparam CLASS_BIT_BRANCH       = 6'd28;
	localparam CLASS_BIT_MODIFY       = 6'd29;
	localparam CLASS_BMOV_BF          = 6'd30;
	localparam CLASS_BF_LOGIC         = 6'd31;
	localparam CLASS_INDIRECT_CMP     = 6'd32;
	localparam CLASS_INDIRECT_MOV     = 6'd33;
	localparam CLASS_MOVM_MASK        = 6'd34;
	localparam CLASS_MUL_RR           = 6'd35;
	localparam CLASS_MUL_IMM          = 6'd36;
	localparam CLASS_DIV_RR           = 6'd37;
	localparam CLASS_DIV_IMM          = 6'd38;
	localparam CLASS_DIRECT_UNARY     = 6'd39;
	localparam CLASS_COMPACT_BYTE_OP  = 6'd40;
	localparam CLASS_STACK_DIRECT     = 6'd41;
	localparam CLASS_INDIRECT_REG_OP  = 6'd42;
	localparam CLASS_RI_BIT_MODIFY    = 6'd43;
	localparam CLASS_BYTE_INDIRECT_OP = 6'd44;
	localparam CLASS_RI_BIT_BRANCH    = 6'd45;
	localparam CLASS_EXTS_DIRECT      = 6'd46;
	localparam CLASS_BTST_DIRECT      = 6'd47;
	localparam CLASS_CALS             = 6'd48;
	localparam CLASS_DM               = 6'd49;

	always @(*) begin
		action_o = ACT_ILL;
		op_class_o = CLASS_NONE;
		op_stage_o = 3'b000;
		next_state_o = ST_FETCH_SETUP;
		target_reg_we_o = 1'b0;
		target_reg_o = 4'h0;
		pair_base_we_o = 1'b0;
		pair_base_o = 4'h0;
		target_addr_we_o = 1'b0;
		target_addr_o = 8'h00;

		case (opcode_i)
			8'h00: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_CLR_FIXED;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h01, 8'h02, 8'h03, 8'h04, 8'h05, 8'h06, 8'h07,
			8'h08, 8'h09, 8'h0A, 8'h0B, 8'h0C, 8'h0D,
			8'h18, 8'h19: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_DIRECT_UNARY;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h0E, 8'h0F, 8'h1E, 8'h1F: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_STACK_DIRECT;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h10, 8'h11, 8'h12, 8'h13, 8'h14, 8'h15, 8'h16, 8'h17: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_COMPACT_BYTE_OP;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h1A, 8'h1B: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_INDIRECT_REG_OP;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h1C, 8'h1D: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_RI_BIT_MODIFY;
				next_state_o = ST_OP_READ8_16H_SETUP;
			end

			8'h20, 8'h21, 8'h22, 8'h23, 8'h24, 8'h25, 8'h26, 8'h27,
			8'h28, 8'h29: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_BYTE_INDIRECT_OP;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h2A, 8'h2B: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_RI_BIT_BRANCH;
				next_state_o = ST_OP_READ8_16H_SETUP;
			end

			8'h2C: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_EXTS_DIRECT;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h2D: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_DM;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h2E: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MOVPS0_IMM;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h2F: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_BTST_DIRECT;
				next_state_o = ST_OP_READ16H_SETUP;
			end

			8'h30: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MEM_CMP8;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h31, 8'h32, 8'h33, 8'h34, 8'h35, 8'h36, 8'h37: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MEM_BYTE_ALU;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h38: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MEM_MOV8;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h39: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MEM_STORE8;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h3A: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MEM_MOVW_LOAD;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h3B: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MEM_MOVW_STORE;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h3C: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MOVW_COMPACT;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h3D: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_DM;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h3E: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_JMP_INDIRECT;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h3F: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_CALL_INDIRECT;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h40, 8'h41, 8'h42, 8'h43, 8'h44, 8'h45, 8'h46, 8'h47,
			8'h48: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_FIXED_BYTE_OP;
				next_state_o = ST_OP_READ16H_SETUP;
			end

			8'h49: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_CALL_ABS;
				next_state_o = ST_OP_READ16H_SETUP;
			end

			8'h4A: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MOVW_DIRECT;
				next_state_o = ST_OP_READ16H_SETUP;
			end

			8'h4B: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MOVW_FIXED;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h4C: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MUL_RR;
				next_state_o = ST_OP_READ16H_SETUP;
			end

			8'h4D: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MUL_IMM;
				next_state_o = ST_OP_READ16H_SETUP;
			end

			8'h4E: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_BMOV_BF;
				next_state_o = ST_OP_READ8_16H_SETUP;
			end

			8'h4F: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_BF_LOGIC;
				next_state_o = ST_OP_READ8_16H_SETUP;
			end

			8'h50, 8'h51, 8'h52, 8'h53, 8'h54, 8'h55, 8'h56, 8'h57,
			8'h58: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_DIRECT_IMM_OP;
				next_state_o = ST_OP_READ16H_SETUP;
			end

			8'h59: action_o = ACT_ILL;

			8'h5A: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_INDIRECT_CMP;
				next_state_o = ST_OP_READ8_16H_SETUP;
			end

			8'h5B: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_INDIRECT_MOV;
				next_state_o = ST_OP_READ8_16H_SETUP;
			end

			8'h5C: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_DIV_RR;
				next_state_o = ST_OP_READ16H_SETUP;
			end

			8'h5D: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_DIV_IMM;
				next_state_o = ST_OP_READ16H_SETUP;
			end

			8'h5E, 8'h5F: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MOVM_MASK;
				next_state_o = ST_OP_READ8_16H_SETUP;
			end

			8'h60, 8'h61, 8'h62, 8'h63, 8'h64, 8'h65, 8'h66, 8'h67: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_WORD_RR_OP;
				next_state_o = ST_OP_READ16H_SETUP;
			end

			8'h68, 8'h69, 8'h6A, 8'h6B, 8'h6C, 8'h6D, 8'h6E, 8'h6F: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_WORD_IMM_OP;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'h70, 8'h71, 8'h72, 8'h73, 8'h74, 8'h75, 8'h76, 8'h77: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_DBNZ;
				next_state_o = ST_OP_READ8_SETUP;
				target_reg_we_o = 1'b1;
				target_reg_o = {1'b0, opcode_i[2:0]};
			end

			8'h78, 8'h79, 8'h7A, 8'h7B, 8'h7C, 8'h7D, 8'h7E, 8'h7F: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MOVW_IMM;
				next_state_o = ST_OP_READ16H_SETUP;
				pair_base_we_o = 1'b1;
				// MOVW RR,#nn numbers its pairs R0,R8,R2,R10,R4,R12,R6,R14, so
				// the low opcode bit picks the upper eight and the other two
				// pick the pair within a half.
				pair_base_o = {opcode_i[0], opcode_i[2:1], 1'b0};
			end

			8'h80, 8'h81, 8'h82, 8'h83, 8'h84, 8'h85, 8'h86, 8'h87,
			8'h88, 8'h89, 8'h8A, 8'h8B, 8'h8C, 8'h8D, 8'h8E, 8'h8F: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_BIT_BRANCH;
				next_state_o = ST_OP_READ8_16H_SETUP;
			end

			8'h90, 8'h91, 8'h92, 8'h93, 8'h94, 8'h95, 8'h96, 8'h97,
			8'h98, 8'h99, 8'h9A, 8'h9B, 8'h9C, 8'h9D, 8'h9E, 8'h9F: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_JMP;
				next_state_o = ST_OP_READ16H_SETUP;
			end

			8'hA0, 8'hA1, 8'hA2, 8'hA3, 8'hA4, 8'hA5, 8'hA6, 8'hA7,
			8'hA8, 8'hA9, 8'hAA, 8'hAB, 8'hAC, 8'hAD, 8'hAE, 8'hAF: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_BIT_MODIFY;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'hB0, 8'hB1, 8'hB2, 8'hB3, 8'hB4, 8'hB5, 8'hB6, 8'hB7: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MOV_FIXED_TO_REG;
				next_state_o = ST_OP_READ8_SETUP;
				target_reg_we_o = 1'b1;
				target_reg_o = {1'b0, opcode_i[2:0]};
			end

			8'hB8, 8'hB9, 8'hBA, 8'hBB, 8'hBC, 8'hBD, 8'hBE, 8'hBF: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MOV_REG_TO_FIXED;
				next_state_o = ST_OP_READ8_SETUP;
				target_reg_we_o = 1'b1;
				target_reg_o = {1'b0, opcode_i[2:0]};
			end

			8'hC0, 8'hC1, 8'hC2, 8'hC3, 8'hC4, 8'hC5, 8'hC6, 8'hC7: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MOVI_REG;
				next_state_o = ST_OP_READ8_SETUP;
				target_reg_we_o = 1'b1;
				target_reg_o = {1'b0, opcode_i[2:0]};
			end

			8'hC8, 8'hC9, 8'hCA, 8'hCB, 8'hCC, 8'hCD, 8'hCE, 8'hCF: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_MOVI_SFR;
				next_state_o = ST_OP_READ8_SETUP;
				target_addr_we_o = 1'b1;
				target_addr_o = {5'b00010, opcode_i[2:0]};
			end

			8'hD0, 8'hD1, 8'hD2, 8'hD3, 8'hD4, 8'hD5, 8'hD6, 8'hD7,
			8'hD8, 8'hD9, 8'hDA, 8'hDB, 8'hDC, 8'hDD, 8'hDE, 8'hDF: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_BR;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'hE0, 8'hE1, 8'hE2, 8'hE3, 8'hE4, 8'hE5, 8'hE6, 8'hE7,
			8'hE8, 8'hE9, 8'hEA, 8'hEB, 8'hEC, 8'hED, 8'hEE, 8'hEF: begin
				action_o = ACT_NORMAL;
				op_class_o = CLASS_CALS;
				next_state_o = ST_OP_READ8_SETUP;
			end

			8'hF0: action_o = ACT_STOP;
			8'hF1: action_o = ACT_HALT;
			8'hF8: action_o = ACT_RET;
			8'hF9: action_o = ACT_IRET;
			8'hFA: action_o = ACT_CLRC;
			8'hFB: action_o = ACT_COMC;
			8'hFC: action_o = ACT_SETC;
			8'hFD: action_o = ACT_EI;
			8'hFE: action_o = ACT_DI;
			8'hFF: action_o = ACT_FETCH;

			default: action_o = ACT_ILL;
		endcase
	end

endmodule
