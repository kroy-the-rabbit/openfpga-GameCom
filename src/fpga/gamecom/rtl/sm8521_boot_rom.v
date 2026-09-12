// Copyright (c) 2026 Jamie Blanks

module sm8521_boot_rom
(
	input wire ce_i,
	input wire [11:0] addr_i,
	output reg [7:0] data_o
);

	integer i;
	(* romstyle = "M10K" *) reg [127:0] rom_q [0:255];
	wire [127:0] line_w = rom_q[addr_i[11:4]];

	initial begin
		for (i = 0; i < 256; i = i + 1) begin
			rom_q[i] = 128'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF;
		end

		rom_q[8'h00] = 128'h1085106A1085106D1070108510851073;
		rom_q[8'h01] = 128'h10851076108510791085107C107F1082;
		rom_q[8'h02] = 128'hFFFFFFFE2E000010001100205846194B;
		rom_q[8'h03] = 128'h1C03C05802255804265803287A10617C;
		rom_q[8'h04] = 128'h202FC10938423044DE1371F8388020F0;
		rom_q[8'h05] = 128'h3180258031804000DE03982054FE9810;
		rom_q[8'h06] = 128'h5E546967657220444D4798203998203C;
		rom_q[8'h07] = 128'h98203F98204298204598204898204B98;
		rom_q[8'h08] = 128'h204E982051F9FFFFFFFFFFFFFFFFFFFF;
	end

	always @(*) begin
		data_o = 8'h00;

		if (ce_i) begin
			case (addr_i[3:0])
				4'h0: data_o = line_w[127:120];
				4'h1: data_o = line_w[119:112];
				4'h2: data_o = line_w[111:104];
				4'h3: data_o = line_w[103:96];
				4'h4: data_o = line_w[95:88];
				4'h5: data_o = line_w[87:80];
				4'h6: data_o = line_w[79:72];
				4'h7: data_o = line_w[71:64];
				4'h8: data_o = line_w[63:56];
				4'h9: data_o = line_w[55:48];
				4'hA: data_o = line_w[47:40];
				4'hB: data_o = line_w[39:32];
				4'hC: data_o = line_w[31:24];
				4'hD: data_o = line_w[23:16];
				4'hE: data_o = line_w[15:8];
				default: data_o = line_w[7:0];
			endcase
		end
	end

endmodule
