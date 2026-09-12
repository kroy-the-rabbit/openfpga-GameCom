// SPDX-License-Identifier: MIT
module gamecom_async_fifo #(
    parameter WIDTH = 54,
    parameter ADDR_BITS = 6
) (
    input wire wr_clk, wr_reset, wr_en,
    input wire [WIDTH-1:0] wr_data,
    output wire wr_full,
    output wire [ADDR_BITS:0] wr_used,
    input wire rd_clk, rd_reset, rd_en,
    output reg [WIDTH-1:0] rd_data,
    output wire rd_empty
);
    localparam PTR_BITS = ADDR_BITS + 1;
    reg [WIDTH-1:0] storage [0:(1<<ADDR_BITS)-1];
    reg [PTR_BITS-1:0] wr_bin, wr_gray, rd_bin, rd_gray;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg [PTR_BITS-1:0] rd_gray_wr1, rd_gray_wr2;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg [PTR_BITS-1:0] wr_gray_rd1, wr_gray_rd2;
    wire [PTR_BITS-1:0] wr_next = wr_bin + 1'b1;
    wire [PTR_BITS-1:0] rd_next = rd_bin + 1'b1;
    function [PTR_BITS-1:0] gray_to_bin(input [PTR_BITS-1:0] gray);
        integer n;
        begin
            gray_to_bin[PTR_BITS-1] = gray[PTR_BITS-1];
            for (n=PTR_BITS-2; n>=0; n=n-1)
                gray_to_bin[n] = gray_to_bin[n+1] ^ gray[n];
        end
    endfunction
    assign wr_full = wr_gray == {~rd_gray_wr2[PTR_BITS-1:PTR_BITS-2], rd_gray_wr2[PTR_BITS-3:0]};
    assign wr_used = wr_bin - gray_to_bin(rd_gray_wr2);
    assign rd_empty = rd_gray == wr_gray_rd2;
    always @(posedge wr_clk) begin
        if (wr_reset) begin
            wr_bin <= 0; wr_gray <= 0;
            rd_gray_wr1 <= 0; rd_gray_wr2 <= 0;
        end else begin
            rd_gray_wr1 <= rd_gray; rd_gray_wr2 <= rd_gray_wr1;
            if (wr_en && !wr_full) begin
                storage[wr_bin[ADDR_BITS-1:0]] <= wr_data;
                wr_bin <= wr_next;
                wr_gray <= (wr_next >> 1) ^ wr_next;
            end
        end
    end
    always @(posedge rd_clk) begin
        if (rd_reset) begin
            rd_bin <= 0; rd_gray <= 0; rd_data <= 0;
            wr_gray_rd1 <= 0; wr_gray_rd2 <= 0;
        end else begin
            wr_gray_rd1 <= wr_gray; wr_gray_rd2 <= wr_gray_rd1;
            if (rd_en && !rd_empty) begin
                rd_data <= storage[rd_bin[ADDR_BITS-1:0]];
                rd_bin <= rd_next;
                rd_gray <= (rd_next >> 1) ^ rd_next;
            end
        end
    end
endmodule
