#!/usr/bin/env python3
"""Elaborate the complete core with clock/IP simulation substitutes; test I2S."""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT / "src/fpga/core"
VENDOR = ROOT / "src/fpga/gamecom"
with tempfile.TemporaryDirectory(prefix="gamecom-integration-") as tmp:
    out = Path(tmp) / "audio"
    subprocess.run(["iverilog", "-g2012", "-s", "tb_i2s", "-o", str(out),
                    str(CORE / "gamecom_i2s.sv"), str(ROOT / "tools/sim/tb_i2s.sv")],check=True)
    subprocess.run(["vvp",str(out)],check=True)
    stub = Path(tmp) / "ip.sv"
    stub.write_text('''module gamecom_pll(input refclk,rst,output clk_mem,clk_sdram,clk_sys,clk_vid,clk_vid_90,locked);
assign {clk_mem,clk_sdram,clk_sys,clk_vid,clk_vid_90}={5{refclk}}; assign locked=!rst; endmodule
module mf_audio_pll(input refclk,rst,output outclk_0,outclk_1,locked);
assign {outclk_0,outclk_1}={2{refclk}}; assign locked=!rst; endmodule
module mf_datatable(input [9:0] address_a,address_b,input clock_a,clock_b,input [31:0] data_a,data_b,
input wren_a,wren_b,output reg [31:0] q_a,q_b);
reg [31:0] mem[0:1023]; always @(posedge clock_a) begin
q_a<=mem[address_a];if(wren_a)mem[address_a]<=data_a;end
always @(posedge clock_b) begin q_b<=mem[address_b];if(wren_b)mem[address_b]<=data_b;end endmodule
''')
    sources = sorted(p for p in CORE.glob("*.sv") if p.name!="gamecom_pll.sv")
    sources += [CORE/"core_bridge_cmd.v",ROOT/"src/fpga/apf/common.v"]
    sources += [VENDOR/"rtl"/f for f in ("gamecom.v","gamecom_audio_output.v","gamecom_input.v","gamecom_video.v","gamecom_cheat_engine.sv","sm8521.v","sm8521_boot_rom.v","Mem/cache_ram.v")]
    subprocess.run(["iverilog","-g2012","-I",str(VENDOR),"-s","core_top","-o",str(Path(tmp)/"top"),str(stub),*map(str,sources)],check=True)
    print("PASS complete Pocket core_top elaboration (PLL and datatable simulation substitutes)")
