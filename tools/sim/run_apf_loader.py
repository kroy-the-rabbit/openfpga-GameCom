#!/usr/bin/env python3
"""Drive APF commands through the actual command-handler and asset-loader RTL."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def main():
    with tempfile.TemporaryDirectory(prefix="gamecom-apf-loader-") as temporary:
        executable = Path(temporary) / "apf_loader.vvp"
        subprocess.run([
            "iverilog", "-g2012", "-s", "tb_apf_loader", "-o", str(executable),
            "src/fpga/core/gamecom_async_fifo.sv", "src/fpga/core/gamecom_rom_loader.sv",
            "src/fpga/core/core_bridge_cmd.v", "src/fpga/apf/common.v", "tools/sim/tb_apf_loader.sv",
        ], cwd=ROOT, check=True)
        subprocess.run(["vvp", str(executable)], cwd=ROOT, check=True, timeout=120)


if __name__ == "__main__":
    main()
