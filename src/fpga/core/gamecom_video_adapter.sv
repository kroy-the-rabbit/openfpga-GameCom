module gamecom_video_adapter (
    input wire clk_vid,
    input wire reset,
    input wire ce_pix,
    input wire hblank,
    input wire vblank,
    input wire hsync,
    input wire vsync,
    input wire [2:0] shade,
    output reg [23:0] video_rgb,
    output reg video_hs,
    output reg video_vs,
    output reg video_de,
    output reg video_skip
);
    reg sample_pending;
    reg hsync_prev, vsync_prev;
    function [23:0] color;
        input [2:0] index;
        begin
            case (index)
                4: color = 24'hE7E8D6;
                3: color = 24'hD4D8BA;
                2: color = 24'hA7AF86;
                1: color = 24'h737D5E;
                default: color = 24'h424B3B;
            endcase
        end
    endfunction
    always @(posedge clk_vid) begin
        if (reset) begin
            sample_pending <= 0;
            hsync_prev <= 1;
            vsync_prev <= 1;
            video_rgb <= 0;
            video_hs <= 0;
            video_vs <= 0;
            video_de <= 0;
            video_skip <= 1;
        end else begin
            sample_pending <= ce_pix;
            video_skip <= !sample_pending;
            video_hs <= 0;
            video_vs <= 0;
            if (sample_pending) begin
                video_rgb <= (hblank || vblank) ? 24'd0 : color(shade);
                video_de <= !hblank && !vblank;
                video_hs <= hsync && !hsync_prev;
                video_vs <= vsync && !vsync_prev;
                hsync_prev <= hsync;
                vsync_prev <= vsync;
            end
        end
    end
endmodule
