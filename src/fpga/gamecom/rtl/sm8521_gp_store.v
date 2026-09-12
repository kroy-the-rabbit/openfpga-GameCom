// Copyright (c) 2026 Jamie Blanks

module sm8521_gp_store
(
	input wire clk_sys_i,
	input wire resetb_i,
	input wire cold_reset_i,
	input wire [4:0] active_bank_i,
	input wire [7:0] write_mask_a_i,
	input wire [63:0] write_data_a_i,
	input wire [7:0] write_mask_b_i,
	input wire [63:0] write_data_b_i,
	output reg init_active_o,
	output wire [127:0] shadow_o,
	input wire ss_active_i,
	input wire [8:0] ss_addr_i,
	input wire ss_wren_i,
	input wire [7:0] ss_wdata_i,
	output reg [7:0] ss_rdata_o
);

	reg [5:0] init_row_q;
	reg [5:0] addr_a_q;
	reg [5:0] addr_b_q;
	reg [7:0] wren_a_q;
	reg [7:0] wren_b_q;
	reg [7:0] wdata_a_q [0:7];
	reg [7:0] wdata_b_q [0:7];
	wire [7:0] rdata_a_w [0:7];
	wire [7:0] rdata_b_w [0:7];
	reg [7:0] shadow_q [0:15];
	integer lane_i;
	integer shadow_i;

	function [7:0] lane_data;
		input [63:0] flat_i;
		input [2:0] lane_sel_i;
		begin
			lane_data = flat_i[{lane_sel_i, 3'b000} +: 8];
		end
	endfunction

	genvar gp_lane_i;
	generate
		for (gp_lane_i = 0; gp_lane_i < 8; gp_lane_i = gp_lane_i + 1) begin: gen_gp_store
			cache_ram_dp #(
				.ADDR_WIDTH(6),
				.DATA_WIDTH(8)
			) u_gp_store (
				.clk_i(clk_sys_i),
				.addr_a_i(addr_a_q),
				.wren_a_i(wren_a_q[gp_lane_i]),
				.wdata_a_i(wdata_a_q[gp_lane_i]),
				.q_a_o(rdata_a_w[gp_lane_i]),
				.addr_b_i(addr_b_q),
				.wren_b_i(wren_b_q[gp_lane_i]),
				.wdata_b_i(wdata_b_q[gp_lane_i]),
				.q_b_o(rdata_b_w[gp_lane_i])
			);
		end
	endgenerate

	assign shadow_o = {
		shadow_q[15], shadow_q[14], shadow_q[13], shadow_q[12],
		shadow_q[11], shadow_q[10], shadow_q[9], shadow_q[8],
		shadow_q[7], shadow_q[6], shadow_q[5], shadow_q[4],
		shadow_q[3], shadow_q[2], shadow_q[1], shadow_q[0]
	};

	always @(*) begin
		case (ss_addr_i[2:0])
			3'd0: ss_rdata_o = rdata_a_w[0];
			3'd1: ss_rdata_o = rdata_a_w[1];
			3'd2: ss_rdata_o = rdata_a_w[2];
			3'd3: ss_rdata_o = rdata_a_w[3];
			3'd4: ss_rdata_o = rdata_a_w[4];
			3'd5: ss_rdata_o = rdata_a_w[5];
			3'd6: ss_rdata_o = rdata_a_w[6];
			default: ss_rdata_o = rdata_a_w[7];
		endcase
	end

	always @(posedge clk_sys_i) begin
		wren_a_q <= 8'h00;
		wren_b_q <= 8'h00;

		if (ss_active_i) begin
			addr_a_q <= ss_addr_i[8:3];
			addr_b_q <= 6'd0;
			wren_a_q <= ss_wren_i ? (8'h01 << ss_addr_i[2:0]) : 8'h00;
			wren_b_q <= 8'h00;
			for (lane_i = 0; lane_i < 8; lane_i = lane_i + 1) begin
				wdata_a_q[lane_i] <= ss_wdata_i;
				wdata_b_q[lane_i] <= 8'h00;
			end
			if (ss_wren_i && (ss_addr_i[8:3] == {1'b0, active_bank_i})) begin
				shadow_q[{1'b0, ss_addr_i[2:0]}] <= ss_wdata_i;
			end
			if (ss_wren_i && (ss_addr_i[8:3] == ({1'b0, active_bank_i} + 6'd1))) begin
				shadow_q[{1'b1, ss_addr_i[2:0]}] <= ss_wdata_i;
			end
		end else if (!resetb_i) begin
			addr_a_q <= 6'd0;
			addr_b_q <= 6'd1;
			if (cold_reset_i) begin
				init_active_o <= 1'b1;
				init_row_q <= 6'd0;
				for (lane_i = 0; lane_i < 8; lane_i = lane_i + 1) begin
					wdata_a_q[lane_i] <= 8'h00;
					wdata_b_q[lane_i] <= 8'h00;
				end
				for (shadow_i = 0; shadow_i < 16; shadow_i = shadow_i + 1) begin
					shadow_q[shadow_i] <= 8'h00;
				end
			end else begin
				init_active_o <= 1'b0;
				init_row_q <= 6'd0;
			end
		end else if (init_active_o) begin
			addr_a_q <= init_row_q;
			addr_b_q <= init_row_q + 6'd1;
			wren_a_q <= 8'hFF;
			if (init_row_q == 6'd32) begin
				wren_b_q <= 8'h00;
			end else begin
				wren_b_q <= 8'hFF;
			end
			for (lane_i = 0; lane_i < 8; lane_i = lane_i + 1) begin
				wdata_a_q[lane_i] <= 8'h00;
				wdata_b_q[lane_i] <= 8'h00;
				shadow_q[lane_i] <= 8'h00;
				shadow_q[lane_i + 8] <= 8'h00;
			end
			if (init_row_q >= 6'd32) begin
				init_active_o <= 1'b0;
			end else begin
				init_row_q <= init_row_q + 6'd2;
			end
		end else begin
			addr_a_q <= {1'b0, active_bank_i};
			addr_b_q <= {1'b0, active_bank_i} + 6'd1;
			for (lane_i = 0; lane_i < 8; lane_i = lane_i + 1) begin
				if (write_mask_a_i[lane_i]) begin
					wren_a_q[lane_i] <= 1'b1;
					wdata_a_q[lane_i] <= lane_data(write_data_a_i, lane_i[2:0]);
					shadow_q[lane_i] <= lane_data(write_data_a_i, lane_i[2:0]);
				end else begin
					shadow_q[lane_i] <= rdata_a_w[lane_i];
				end

				if (write_mask_b_i[lane_i]) begin
					wren_b_q[lane_i] <= 1'b1;
					wdata_b_q[lane_i] <= lane_data(write_data_b_i, lane_i[2:0]);
					shadow_q[lane_i + 8] <= lane_data(write_data_b_i, lane_i[2:0]);
				end else begin
					shadow_q[lane_i + 8] <= rdata_b_w[lane_i];
				end
			end
		end
	end

`ifndef SYNTHESIS
	function [7:0] debug_bank_read;
		input [8:0] idx;
		begin
			case (idx[2:0])
				3'd0: debug_bank_read = gen_gp_store[0].u_gp_store.mem_q[idx[8:3]];
				3'd1: debug_bank_read = gen_gp_store[1].u_gp_store.mem_q[idx[8:3]];
				3'd2: debug_bank_read = gen_gp_store[2].u_gp_store.mem_q[idx[8:3]];
				3'd3: debug_bank_read = gen_gp_store[3].u_gp_store.mem_q[idx[8:3]];
				3'd4: debug_bank_read = gen_gp_store[4].u_gp_store.mem_q[idx[8:3]];
				3'd5: debug_bank_read = gen_gp_store[5].u_gp_store.mem_q[idx[8:3]];
				3'd6: debug_bank_read = gen_gp_store[6].u_gp_store.mem_q[idx[8:3]];
				default: debug_bank_read = gen_gp_store[7].u_gp_store.mem_q[idx[8:3]];
			endcase
		end
	endfunction

	task debug_bank_write;
		input [8:0] idx;
		input [7:0] data;
		reg [8:0] active_base_v;
		reg [3:0] shadow_idx_v;
		begin
			case (idx[2:0])
				3'd0: gen_gp_store[0].u_gp_store.mem_q[idx[8:3]] = data;
				3'd1: gen_gp_store[1].u_gp_store.mem_q[idx[8:3]] = data;
				3'd2: gen_gp_store[2].u_gp_store.mem_q[idx[8:3]] = data;
				3'd3: gen_gp_store[3].u_gp_store.mem_q[idx[8:3]] = data;
				3'd4: gen_gp_store[4].u_gp_store.mem_q[idx[8:3]] = data;
				3'd5: gen_gp_store[5].u_gp_store.mem_q[idx[8:3]] = data;
				3'd6: gen_gp_store[6].u_gp_store.mem_q[idx[8:3]] = data;
				default: gen_gp_store[7].u_gp_store.mem_q[idx[8:3]] = data;
			endcase

			active_base_v = {1'b0, active_bank_i, 3'b000};
			if ((idx >= active_base_v) && (idx < (active_base_v + 9'd16))) begin
				shadow_idx_v = idx[3:0] - active_base_v[3:0];
				shadow_q[shadow_idx_v] = data;
			end
		end
	endtask
`endif

endmodule
