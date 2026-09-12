#!/usr/bin/env python3
"""RTL tests with the same ROM asset path used by the Quartus project."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT / "src/fpga/core"
VENDOR = ROOT / "src/fpga/gamecom"
BUILD = ROOT / "src/fpga/build"
BUILD.mkdir(parents=True, exist_ok=True)
CASES = {
    "input": [CORE / "gamecom_pocket_input.sv"],
    "rtc": [CORE / "gamecom_rtc.sv"],
    "video": [CORE / "gamecom_video_adapter.sv", VENDOR / "rtl/gamecom_video.v", VENDOR / "rtl/Mem/cache_ram.v"],
}

with tempfile.TemporaryDirectory(prefix="gamecom-machine-") as tmp:
    for name, sources in CASES.items():
        output = Path(tmp) / name
        subprocess.run(["iverilog", "-g2012", "-s", f"gamecom_{name}_tb", "-o", str(output),
                        *map(str, sources), str(ROOT / f"tools/sim/gamecom_{name}_tb.sv")], check=True, cwd=BUILD)
        subprocess.run(["vvp", str(output)], check=True, cwd=BUILD, timeout=120)
    sources = [CORE / name for name in ("gamecom_machine.sv", "gamecom_pocket_input.sv", "gamecom_rtc.sv", "gamecom_video_adapter.sv")]
    sources += [VENDOR / "rtl" / name for name in ("gamecom.v", "gamecom_audio_output.v", "gamecom_input.v", "gamecom_video.v", "gamecom_cheat_engine.sv", "sm8521.v", "sm8521_boot_rom.v", "Mem/cache_ram.v")]
    subprocess.run(["iverilog", "-g2012", "-I", str(VENDOR), "-s", "gamecom_machine_tb", "-o", str(Path(tmp)/"machine"),
                    *map(str,sources), str(ROOT / "tools/sim/gamecom_machine_tb.sv")],check=True,cwd=BUILD)
    subprocess.run(["vvp",str(Path(tmp)/"machine")],check=True,cwd=BUILD,timeout=120)
