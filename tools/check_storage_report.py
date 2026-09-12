#!/usr/bin/env python3
"""Validate storage test results against source and ROM hashes."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
SIZES = {0x8000, 0x40000, 0x80000, 0x100000, 0x1c0000, 0x200000}
SOURCES = {
    "src/fpga/core/gamecom_async_fifo.sv",
    "src/fpga/core/gamecom_rom_loader.sv",
    "src/fpga/core/gamecom_memory.sv",
    "src/fpga/core/gamecom_sdram.sv",
    "tools/sim/gamecom_memory_models.sv",
    "tools/sim/tb_gamecom_storage.sv",
    "tools/sim/run_storage_integration.py",
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def check(report, corpus, count):
    require(report["corpus"] == corpus, "Wrong corpus type")
    require(report["expected_images"] == count, "Wrong expected image count")
    require(report["completed_images"] == count, "Incomplete image results")
    require(report["bridge_clocks_per_word"] == 75, "Wrong APF transfer cadence")
    rows = report["results"]
    require(len(rows) == count, "Wrong number of result rows")
    require({row["index"] for row in rows} == set(range(count)), "Missing or duplicate case indexes")
    require({row["rom_bytes"] for row in rows} == SIZES, "Not all six ROM sizes were checked")
    require(len({row["rom_sha256"] for row in rows}) == count, "Duplicate ROM payloads")
    require(len({row["bios_sha256"] for row in rows}) == 1, "Inconsistent BIOS fixture")
    if corpus == "private":
        manifest = json.loads((ROOT / "tools/fixtures/roms.json").read_text())
        expected_roms = {member["sha256"] for member in manifest["roms"]}
        require({row["rom_sha256"] for row in rows} == expected_roms,
                "Private results do not cover the recorded ROM corpus")
    hashes = report["source_sha256"]
    require(set(hashes) == SOURCES, "Missing or unexpected source provenance")
    for source in sorted(SOURCES):
        actual = hashlib.sha256((ROOT / source).read_bytes()).hexdigest()
        require(hashes[source] == actual, f"Source changed since compilation: {source}")
    for row in rows:
        label = f"case {row['index']} ({row.get('title', 'unnamed')})"
        require(row["status"] == "pass" and row["exit_code"] == 0, f"Failed {label}")
        for kind in ("bios", "rom"):
            expected = row[f"expected_{kind}_crc32"]
            physical = row[f"physical_{kind}_crc32"]
            digest = row[f"{kind}_sha256"]
            require(isinstance(expected, str) and re.fullmatch(r"[0-9a-f]{8}", expected),
                    f"Invalid {kind} CRC in {label}")
            require(expected == physical, f"Physical {kind} CRC mismatch in {label}")
            require(isinstance(digest, str) and re.fullmatch(r"[0-9a-f]{64}", digest),
                    f"Invalid {kind} hash in {label}")
        require(0 <= row["fifo_high_water"] < 64, f"Invalid FIFO occupancy in {label}")
        require(0 < row["sdram_phase_ps"] < row["memory_period_ps"],
                f"Missing memory timing metadata in {label}")
    print(f"PASS: {count} unique {corpus} images, all six sizes, matching physical CRCs and current source hashes")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--corpus", choices=("synthetic", "private"), required=True)
    parser.add_argument("--expected-images", type=int, required=True)
    args = parser.parse_args()
    try:
        check(json.loads(args.report.read_text()), args.corpus, args.expected_images)
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(f"Storage report rejected: {error}") from error


if __name__ == "__main__":
    main()
