module cache_ram
#(
	parameter ADDR_WIDTH = 7,
	parameter DATA_WIDTH = 32
)
(
	input  wire                  clk_i,
	input  wire [ADDR_WIDTH-1:0] addr_i,
	input  wire                  wren_i,
	input  wire [DATA_WIDTH-1:0] wdata_i,
	output wire [DATA_WIDTH-1:0] q_o
);

`ifdef ALTERA_RESERVED_QIS
	spram #(
		.ADDR_WIDTH(ADDR_WIDTH),
		.DATA_WIDTH(DATA_WIDTH)
	) u_spram (
		.clock    (clk_i),
		.address  (addr_i),
		.wren     (wren_i),
		.data     (wdata_i),
		.q        (q_o)
	);

`else
	reg [DATA_WIDTH-1:0] q_out;
	localparam MEM_DEPTH = (1 << ADDR_WIDTH);

	(* ramstyle = "M10K, no_rw_check" *) reg [DATA_WIDTH-1:0] mem_q [0:MEM_DEPTH-1];

	always @(posedge clk_i) begin
		if (wren_i) begin
			mem_q[addr_i] <= wdata_i;
		end

		if (wren_i) begin
			q_out <= wdata_i;
		end else begin
			q_out <= mem_q[addr_i];
		end
	end

	assign q_o = q_out;
`endif

endmodule

module cache_ram_dp
#(
	parameter ADDR_WIDTH = 7,
	parameter DATA_WIDTH = 32,
	parameter CROSS_PORT_FORWARD = 1'b0
)
(
	input  wire                  clk_i,
	input  wire [ADDR_WIDTH-1:0] addr_a_i,
	input  wire                  wren_a_i,
	input  wire [DATA_WIDTH-1:0] wdata_a_i,
	output wire [DATA_WIDTH-1:0] q_a_o,
	input  wire [ADDR_WIDTH-1:0] addr_b_i,
	input  wire                  wren_b_i,
	input  wire [DATA_WIDTH-1:0] wdata_b_i,
	output wire [DATA_WIDTH-1:0] q_b_o
);

`ifdef ALTERA_RESERVED_QIS
	dpram #(
		.ADDR_WIDTH(ADDR_WIDTH),
		.DATA_WIDTH(DATA_WIDTH)
	) u_dpram (
		.clock     (clk_i),

		.address_a (addr_a_i),
		.wren_a    (wren_a_i),
		.data_a    (wdata_a_i),
		.q_a       (q_a_o),

		.address_b (addr_b_i),
		.wren_b    (wren_b_i),
		.data_b    (wdata_b_i),
		.q_b       (q_b_o)
	);

`else
	reg [DATA_WIDTH-1:0] q_a_out;
	reg [DATA_WIDTH-1:0] q_b_out;
	localparam MEM_DEPTH = (1 << ADDR_WIDTH);

	// Default to no modeled cross-port read-during-write forwarding. Some
	// callers, like VRAM, intentionally rely on leaving that collision undefined.
	(* ramstyle = "M10K, no_rw_check" *) reg [DATA_WIDTH-1:0] mem_q [0:MEM_DEPTH-1];

	always @(posedge clk_i) begin
		if (wren_a_i) begin
			mem_q[addr_a_i] <= wdata_a_i;
		end
		if (wren_a_i) begin
			q_a_out <= wdata_a_i;
		end else if (CROSS_PORT_FORWARD && wren_b_i && (addr_b_i == addr_a_i)) begin
			q_a_out <= wdata_b_i;
		end else begin
			q_a_out <= mem_q[addr_a_i];
		end

		if (wren_b_i) begin
			mem_q[addr_b_i] <= wdata_b_i;
		end
		if (wren_b_i) begin
			q_b_out <= wdata_b_i;
		end else if (CROSS_PORT_FORWARD && wren_a_i && (addr_a_i == addr_b_i)) begin
			q_b_out <= wdata_a_i;
		end else begin
			q_b_out <= mem_q[addr_b_i];
		end
	end

	assign q_a_o = q_a_out;
	assign q_b_o = q_b_out;
`endif

endmodule
