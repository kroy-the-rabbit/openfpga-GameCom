#!/usr/bin/env python3
"""Optional actual-ROM CPU/video smoke test. Assets are never checked in."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import zipfile
import struct
import zlib

ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT / "src/fpga/core"
VENDOR = ROOT / "src/fpga/gamecom"
WORK = ROOT / "src/fpga/build"
OUT = ROOT / "build/sim"
p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--romset", type=Path, default=os.environ.get("ROMSET"))
p.add_argument("--rom", default="Lights Out")
p.add_argument("--milliseconds", type=int, default=1000)
p.add_argument("--jobs", type=int, default=4)
p.add_argument("--output-dir", type=Path, default=OUT, help="Isolated build, capture and report directory (default: build/sim)")
p.add_argument("--keys", default="", help="Comma-separated milliseconds:hex-mask controller events, e.g. 6000:10,6200:0")
p.add_argument("--rom-wait-cycles", type=int, default=0, help="20 MHz clock delay on each new cartridge word (BIOS remains asynchronous)")
p.add_argument("--launch-cartridge", action="store_true", help="Use R+D-pad then R+A on the BIOS cartridge icon after the eight-second intro")
p.add_argument("--live-keys", type=Path, help="Diagnostic control file: auto, a hexadecimal key mask, or stop; applied events are recorded for replay")
p.add_argument("--expect-frame-fnv", type=lambda value: int(value,16), help="Fail unless the final RGB frame has this hexadecimal FNV-1a checksum")
p.add_argument("--synthesis", action="store_true", help="Define SYNTHESIS to exercise the CPU branch used by the Quartus project")
args = p.parse_args()
OUT = args.output_dir.resolve()
if args.milliseconds < 1 or args.jobs < 1 or args.rom_wait_cycles < 0:
    p.error("--milliseconds and --jobs must be positive; --rom-wait-cycles must be nonnegative")
if args.expect_frame_fnv is not None and not 0 <= args.expect_frame_fnv <= 0xffffffff:
    p.error("--expect-frame-fnv must fit in 32 bits")
if args.romset is None or shutil.which("verilator") is None:
    print("SKIP actual ROM boot: requires ROMSET/--romset and Verilator")
    raise SystemExit(0)
OUT.mkdir(parents=True,exist_ok=True)
sources = [CORE / n for n in ("gamecom_machine.sv", "gamecom_pocket_input.sv", "gamecom_rtc.sv", "gamecom_video_adapter.sv")]
sources += [VENDOR / "rtl" / n for n in ("gamecom.v", "gamecom_audio_output.v", "gamecom_input.v", "gamecom_video.v", "gamecom_cheat_engine.sv", "sm8521.v", "sm8521_boot_rom.v", "Mem/cache_ram.v")]
hashed_files=set(sources)|{ROOT/"tools/sim/gamecom_boot_sim.sv",ROOT/"tools/sim/gamecom_boot_main.cpp",Path(__file__).resolve()}
hashed_files.update(p for p in (VENDOR/"rtl").rglob("*") if p.is_file())
source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(hashed_files)}
command=["verilator", *(["-DSYNTHESIS"] if args.synthesis else []), "--cc", "--exe", "--build", "-j", str(args.jobs), "-O3", "-CFLAGS", "-O3", "-Wno-fatal", "--top-module", "gamecom_boot_sim", "-I"+str(VENDOR), "--Mdir", str(OUT/"boot-obj"), str(ROOT/"tools/sim/gamecom_boot_sim.sv"), *map(str,sources), str(ROOT/"tools/sim/gamecom_boot_main.cpp")]
with (OUT/"rom-boot-build.log").open("w") as log:
    result=subprocess.run(command,cwd=WORK,stdout=log,stderr=subprocess.STDOUT)
if result.returncode:
    raise SystemExit(f"Verilator build failed; see {OUT/'rom-boot-build.log'}")
with zipfile.ZipFile(args.romset) as archive, tempfile.TemporaryDirectory(prefix="gamecom-rom-boot-") as tmp:
    bios=[n for n in archive.namelist() if "External BIOS" in n and "[!]" in n]
    rom=[n for n in archive.namelist() if args.rom.lower() in n.lower() and n.lower().endswith(".tgc")]
    if len(bios)!=1 or len(rom)!=1:
        raise SystemExit(f"Need one exact BIOS/ROM match, got BIOS={bios!r}, ROM={rom!r}")
    bpath=Path(tmp)/"bios.bin";rpath=Path(tmp)/"game.tgc"
    events=Path(tmp)/"keys.csv"
    key_events=([(8050,0x204),(8080,0x200),(8110,0x204),(8140,0x200),
                 (8170,0x204),(8200,0x200),(8230,0x204),(8260,0x200),
                 (8290,0x201),(8320,0x200),(8350,0x201),(8380,0x200),
                 (8420,0x210),(8520,0)] if args.launch_cartridge else [])
    for item in filter(None,args.keys.split(",")):
        ms,mask=item.split(":")
        key_events.append((int(ms),int(mask,16)))
    key_events.sort()
    if len({ms for ms,_ in key_events})!=len(key_events):
        raise SystemExit("Controller events must have unique times")
    events.write_text("".join(f"{ms} {mask:x}\n" for ms,mask in key_events))
    bpath.write_bytes(archive.read(bios[0]));rpath.write_bytes(archive.read(rom[0]))
    result=subprocess.run([str(OUT/"boot-obj/Vgamecom_boot_sim"),str(bpath),str(rpath),str(args.milliseconds),str(OUT/"rom-boot-frame.ppm"),str(events),str(args.rom_wait_cycles),str(args.live_keys.resolve()) if args.live_keys else ""],cwd=WORK,text=True,capture_output=True)
    print(result.stdout,end="");print(result.stderr,end="")
    (OUT/"rom-boot-run.log").write_text(result.stdout+result.stderr)
    if not result.stdout.strip():
        raise SystemExit(result.returncode or "Simulator produced no result; see rom-boot-run.log")
    report={"rom":rom[0],"rom_size_bytes":rpath.stat().st_size,"bios":bios[0],"rom_sha256":hashlib.sha256(rpath.read_bytes()).hexdigest(),
            "bios_sha256":hashlib.sha256(bpath.read_bytes()).hexdigest(),"source_sha256":source_hashes,
            "compiler_defines":["SYNTHESIS"] if args.synthesis else [],"compiler_command":command,
            "keys":key_events,"simulation":"behavioral ROM memory, configurable cartridge READY delay","result":json.loads(result.stdout.splitlines()[-1])}
    (OUT/"rom-boot-report.json").write_text(json.dumps(report,indent=2)+"\n")
    for ppm in sorted(OUT.glob("rom-boot-frame*.ppm")):
        header, width, maximum, rgb = ppm.read_bytes().split(b"\n",3)
        assert (header,width,maximum)==(b"P6",b"200 160",b"255")
        assert len(rgb)==200*160*3
        def chunk(kind,data):
            return struct.pack(">I",len(data))+kind+data+struct.pack(">I",zlib.crc32(kind+data))
        scanlines=b"".join(b"\0"+rgb[y*600:(y+1)*600] for y in range(160))
        png=b"\x89PNG\r\n\x1a\n"+chunk(b"IHDR",struct.pack(">IIBBBBB",200,160,8,2,0,0,0))
        png+=chunk(b"IDAT",zlib.compress(scanlines))+chunk(b"IEND",b"")
        ppm.with_suffix(".png").write_bytes(png)
    if args.expect_frame_fnv is not None and int(report["result"]["frame_fnv1a"],16) != args.expect_frame_fnv:
        raise SystemExit(f"Final frame checksum mismatch: expected {args.expect_frame_fnv:08x}, got {report['result']['frame_fnv1a']}")
    raise SystemExit(result.returncode)
