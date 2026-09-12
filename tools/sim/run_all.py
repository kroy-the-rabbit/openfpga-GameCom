#!/usr/bin/env python3
"""Run the complete public simulation suite; ROMSET enables private corpus tests."""
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[2]
extended = {"run_all.py", "run_storage_integration.py", "run_rom_boot.py"}
runners = sorted(p for p in (root / "tools/sim").glob("run_*.py") if p.name not in extended)
if not runners:
    raise SystemExit("No test runners found")
for runner in runners:
    print(f"\n== {runner.name}", flush=True)
    subprocess.run([sys.executable, str(runner)], cwd=root, check=True)
print(f"PASS: {len(runners)} test suites")
