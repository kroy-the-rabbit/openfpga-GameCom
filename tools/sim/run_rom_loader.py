#!/usr/bin/env python3
"""Exercise the APF loader with unrelated clocks and a variable-latency backend."""
from pathlib import Path
import re
import subprocess
import tempfile
import zlib

ROOT = Path(__file__).resolve().parents[2]


def main():
    with tempfile.TemporaryDirectory(prefix="gamecom-loader-") as temporary:
        executable = Path(temporary) / "loader.vvp"
        subprocess.run([
            "iverilog", "-g2012", "-s", "tb_rom_loader", "-o", str(executable),
            "src/fpga/core/gamecom_async_fifo.sv", "src/fpga/core/gamecom_rom_loader.sv",
            "tools/sim/tb_rom_loader.sv",
        ], cwd=ROOT, check=True)
        try:
            result = subprocess.run(["vvp", str(executable)], cwd=ROOT, text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
        except subprocess.TimeoutExpired as exception:
            print(exception.stdout or "", flush=True)
            raise
        print(result.stdout, end="", flush=True)
        result.check_returncode()
        verified = re.findall(r"VERIFIED ([0-9a-f]+) ([0-9a-f]+)", result.stdout)
        assert len(verified) == 5, f"expected five verified transfers, got {len(verified)}"
        for length_hex, crc_hex in verified:
            length = int(length_hex, 16)
            data = b"".join(((0x01234567 ^ (offset * 0x01010101)) & 0xFFFFFFFF).to_bytes(4, "big")
                            for offset in range(0, length, 4))
            assert zlib.crc32(data) == int(crc_hex, 16), "readback differs from independent CRC oracle"
        print("PASS independent zlib CRC oracle for all five transfers")


if __name__ == "__main__":
    main()
