#!/usr/bin/env python3
"""Real SM8521 DMA consumers must honor delayed ROM data in all sample paths."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
RTL=ROOT/"src/fpga/gamecom"


def main():
    cpu=Path(os.environ.get("DMA_CPU_SOURCE",RTL/"rtl/sm8521.v"))
    sources=[cpu,RTL/"rtl/sm8521_boot_rom.v",RTL/"rtl/gamecom_cheat_engine.sv",
             RTL/"rtl/Mem/cache_ram.v",ROOT/"tools/sim/tb_gamecom_dma_wait.sv"]
    with tempfile.TemporaryDirectory(prefix="gamecom-dma-") as temp:
        executable=Path(temp)/"dma.vvp"
        subprocess.run(["iverilog","-g2012","-s","tb_gamecom_dma_wait","-I",str(RTL),
                        "-o",str(executable),*map(str,sources)],check=True)
        subprocess.run(["vvp",str(executable)],check=True)


if __name__=="__main__":
    main()
