// SPDX-License-Identifier: GPL-3.0-or-later
module gamecom_pll (
    input wire refclk, input wire rst,
    output wire clk_mem, clk_sdram, clk_sys, clk_vid, clk_vid_90,
    output wire locked
);
    altera_pll #(
        .fractional_vco_multiplier("true"),
        .reference_clock_frequency("74.25 MHz"),
        .operation_mode("normal"), .number_of_clocks(5),
        .output_clock_frequency0("60.0 MHz"), .phase_shift0("0 ps"), .duty_cycle0(50),
        .output_clock_frequency1("60.0 MHz"), .phase_shift1("8333 ps"), .duty_cycle1(50),
        .output_clock_frequency2("20.0 MHz"), .phase_shift2("0 ps"), .duty_cycle2(50),
        .output_clock_frequency3("30.0 MHz"), .phase_shift3("0 ps"), .duty_cycle3(50),
        .output_clock_frequency4("30.0 MHz"), .phase_shift4("8333 ps"), .duty_cycle4(50),
        .pll_type("General"), .pll_subtype("General")
    ) pll_i (
        .refclk(refclk), .rst(rst), .fbclk(1'b0), .fboutclk(),
        .outclk({clk_vid_90, clk_vid, clk_sys, clk_sdram, clk_mem}), .locked(locked)
    );
endmodule
