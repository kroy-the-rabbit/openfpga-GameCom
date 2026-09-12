// SPDX-License-Identifier: GPL-3.0-or-later
module gamecom_i2s (
    input wire clk_sys, reset_sys,
    input wire signed [15:0] sample,
    input wire clk_audio, reset_audio,
    output reg audio_lrck, output reg audio_dac
);
    reg request = 0;
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED" *) reg [2:0] req_sync = 0, ack_sync = 0;
    reg acknowledge = 0;
    reg [15:0] held_sample = 0;
    reg [15:0] next_sample = 0, frame_sample = 0;
    reg [5:0] bit_count = 0;
    always @(posedge clk_sys) begin
        req_sync <= {req_sync[1:0],request};
        if (reset_sys) begin
            req_sync <= 0; acknowledge <= 0; held_sample <= 0;
        end else if (req_sync[2] != acknowledge) begin
            held_sample <= sample;
            acknowledge <= req_sync[2];
        end
    end
    always @(posedge clk_audio) begin
        ack_sync <= {ack_sync[1:0],acknowledge};
        if (reset_audio) begin
            ack_sync <= 0; request <= 0; bit_count <= 0;
            next_sample <= 0; frame_sample <= 0;
            audio_lrck <= 0; audio_dac <= 0;
        end else begin
            if (ack_sync[2] == request) next_sample <= held_sample;
            bit_count <= bit_count + 1'b1;
            if (bit_count == 0) begin
                audio_lrck <= 0; frame_sample <= next_sample;
                if (ack_sync[2] == request) request <= ~request;
            end
            if (bit_count == 32) audio_lrck <= 1;
            if ((bit_count[4:0] >= 1) && (bit_count[4:0] <= 16))
                audio_dac <= frame_sample[16 - bit_count[4:0]];
            else audio_dac <= 0;
        end
    end
endmodule
