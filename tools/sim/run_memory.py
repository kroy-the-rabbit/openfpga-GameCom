#!/usr/bin/env python3
"""Exercise real memory RTL against timed SRAM and SDRAM pin models."""
import hashlib
import os
from pathlib import Path
import random
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[2]


def corpus_fixture(path: Path, archive: str) -> int:
    seen = set()
    probes = 0
    rng = random.Random(0x8521)
    with zipfile.ZipFile(archive) as roms, path.open("w") as output:
        for entry in roms.infolist():
            if entry.is_dir():
                continue
            is_rom = entry.filename.lower().endswith(".tgc")
            is_bios = "bios" in entry.filename.lower() and entry.file_size == 0x40000
            if not (is_rom or is_bios):
                continue
            data = roms.read(entry)
            digest = hashlib.sha256(data).digest()
            if digest in seen:
                continue
            seen.add(digest)
            size = len(data)
            if is_rom and size not in (0x8000, 0x40000, 0x80000, 0x100000, 0x1c0000, 0x200000):
                raise RuntimeError(f"Unsupported private ROM size: {entry.filename}")
            addresses = {0, size-4}
            addresses.update(rng.randrange(size//4)*4 for _ in range(16))
            addresses.update(a for a in (0x7ffc,0x8000,0x3fffc,0x40000,0x7fffc,0x80000,0xffffc,0x100000) if a<size)
            for offset in sorted(addresses):
                backing = offset + (0x40000 if is_rom and size==0x1c0000 else 0)
                physical = backing
                if is_rom and physical < 0x40000:
                    if size > 0x40000:
                        continue
                    physical += 0x40000
                value = int.from_bytes(data[offset:offset+4], "big")
                output.write(f"{int(is_bios)} {size:x} {backing:x} {physical:x} {value:08x}\n")
                probes += 1
    return probes


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="gamecom-memory-") as temp:
        executable = Path(temp)/"memory.vvp"
        sources = ["src/fpga/core/gamecom_memory.sv", "src/fpga/core/gamecom_sdram.sv",
                   "tools/sim/gamecom_memory_models.sv", "tools/sim/tb_gamecom_memory.sv"]
        args = ["vvp", str(executable)]
        if archive := os.environ.get("ROMSET"):
            fixture = Path(temp)/"private-probes.txt"
            count = corpus_fixture(fixture, archive)
            print(f"Private corpus: {count} four-byte physical memory probes", flush=True)
            args.append(f"+CORPUS={fixture}")
        for period_ns, system_ns, phase_ns in ((16.666667, 50.0, 8.333333), (16.650, 49.950, 8.325)):
            for access_ns in (6.0, 2.5):
                subprocess.run(["iverilog", "-g2012", "-Wall", "-s", "tb_gamecom_memory",
                                f"-Ptb_gamecom_memory.SDRAM_ACCESS_NS={access_ns}",
                                f"-Ptb_gamecom_memory.MEMORY_PERIOD_NS={period_ns}",
                                f"-Ptb_gamecom_memory.SYSTEM_PERIOD_NS={system_ns}",
                                f"-Ptb_gamecom_memory.SDRAM_PHASE_NS={phase_ns}",
                                "-o", str(executable), *map(str, (ROOT/s for s in sources))], check=True)
                subprocess.run(args, check=True)


if __name__ == "__main__":
    main()
