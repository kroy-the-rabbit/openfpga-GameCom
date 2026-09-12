// SPDX-License-Identifier: GPL-3.0-or-later
module gamecom_reset_sync(input wire clk, input wire async_reset, output wire reset);
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *)
    reg [2:0] reset_pipe = 3'b111;
    always @(posedge clk or posedge async_reset)
        if (async_reset) reset_pipe <= 3'b111;
        else reset_pipe <= {reset_pipe[1:0],1'b0};
    assign reset = reset_pipe[2];
endmodule
