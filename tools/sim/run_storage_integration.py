#!/usr/bin/env python3
"""Full image load and physical CRC, all six layouts at APF transfer cadence."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import shutil
import tempfile
import time
import zipfile
import zlib

ROOT = Path(__file__).resolve().parents[2]
SIZES = (0x8000,0x40000,0x80000,0x100000,0x1c0000,0x200000)


def synthetic(size: int, seed: int) -> bytes:
    return bytes((i ^ (i>>8) ^ (i>>16) ^ seed) & 255 for i in range(size))


def word_file(path: Path, data: bytes) -> None:
    with path.open("w") as output:
        output.writelines(data[i:i+4].hex()+"\n" for i in range(0,len(data),4))


def images(sizes):
    if archive := os.environ.get("ROMSET"):
        with zipfile.ZipFile(archive) as roms:
            bios_files=[e for e in roms.infolist() if "bios" in e.filename.lower() and e.file_size==0x40000]
            bios_files.sort(key=lambda e: ("[!]" not in e.filename,e.filename))
            if not bios_files:
                raise RuntimeError("ROMSET has no external 256KiB BIOS")
            bios=roms.read(bios_files[0]); selected=set(); seen=set()
            for entry in roms.infolist():
                if not entry.filename.lower().endswith(".tgc") or entry.file_size not in sizes:
                    continue
                if entry.file_size in selected and os.environ.get("ROMSET_ALL")!="1":
                    continue
                data=roms.read(entry); digest=hashlib.sha256(data).digest()
                if digest in seen:
                    continue
                selected.add(len(data));seen.add(digest)
                yield entry.filename,bios,data
            if selected!=set(sizes):
                raise RuntimeError(f"ROMSET missing requested size classes: {set(sizes)-selected}")
    else:
        bios=synthetic(0x40000,0xb1)
        for size in sizes:
            yield f"synthetic {size:#x}",bios,synthetic(size,0x65)


def main() -> None:
    sizes=tuple(int(value,16) for value in os.environ.get("STORAGE_SIZES","").split(",") if value) or SIZES
    if not set(sizes)<=set(SIZES):
        raise RuntimeError("STORAGE_SIZES contains an unsupported length")
    jobs=int(os.environ.get("STORAGE_JOBS","1"))
    if not 1<=jobs<=8:
        raise RuntimeError("STORAGE_JOBS must be between 1 and 8")
    sources=["src/fpga/core/gamecom_async_fifo.sv","src/fpga/core/gamecom_rom_loader.sv",
             "src/fpga/core/gamecom_memory.sv","src/fpga/core/gamecom_sdram.sv",
             "tools/sim/gamecom_memory_models.sv","tools/sim/tb_gamecom_storage.sv"]
    source_hashes={source:hashlib.sha256((ROOT/source).read_bytes()).hexdigest()
                   for source in (*sources,"tools/sim/run_storage_integration.py")}
    with tempfile.TemporaryDirectory(prefix="gamecom-storage-") as temp:
        temp=Path(temp); executable=temp/"storage.vvp"
        simulator=os.environ.get("STORAGE_SIM") or ("verilator" if shutil.which("verilator") else "iverilog")
        if simulator=="verilator":
            build=temp/"verilated"
            compile_command=["verilator","--binary","--timing","-Wno-fatal","-j","2",
                             "--top-module","tb_gamecom_storage","--Mdir",str(build),
                             *map(str,(ROOT/s for s in sources))]
            executable=build/"Vtb_gamecom_storage"
            run_command=[str(executable)]
        elif simulator=="iverilog":
            compile_command=["iverilog","-g2012","-s","tb_gamecom_storage","-o",str(executable),
                             *map(str,(ROOT/s for s in sources))]
            run_command=["vvp",str(executable)]
        else:
            raise RuntimeError("STORAGE_SIM must be verilator or iverilog")
        compiled=subprocess.run(compile_command,text=True,capture_output=True)
        if compiled.returncode:
            print(compiled.stdout+compiled.stderr,flush=True)
            compiled.check_returncode()
        print(f"Storage integration simulator: {simulator}",flush=True)
        version=subprocess.run([simulator,"--version"] if simulator=="verilator" else ["iverilog","-V"],
                               text=True,capture_output=True,check=True).stdout.splitlines()[0]

        def run_case(case):
            index,(title,bios,rom)=case
            private=temp/f"image-{index:02d}"
            private.mkdir()
            bios_file=private/"bios.hex";rom_file=private/"rom.hex"
            word_file(bios_file,bios);word_file(rom_file,rom)
            print(f"Storage integration: {title} ({len(rom):#x}); full BIOS and ROM CRC",flush=True)
            started=time.monotonic()
            completed=subprocess.run([*run_command,f"+BIOS={bios_file}",f"+ROM={rom_file}",
                                      f"+SIZE={len(rom):x}",f"+BIOSCRC={zlib.crc32(bios):08x}",
                                      f"+ROMCRC={zlib.crc32(rom):08x}"],text=True,capture_output=True)
            elapsed=time.monotonic()-started
            log=completed.stdout+completed.stderr
            match=re.search(r"PASS storage integration size=([0-9a-f]+) physical CRC=([0-9a-f]+) FIFO high-water=(\d+) phase_ps=(\d+) period_ps=(\d+)",log)
            bios_match=re.search(r"PASS BIOS physical CRC=([0-9a-f]+)",log)
            passed=completed.returncode==0 and match is not None and bios_match is not None
            if passed:
                passed=(int(match[1],16)==len(rom) and int(match[2],16)==zlib.crc32(rom)
                        and int(bios_match[1],16)==zlib.crc32(bios))
            return {
                "index":index,"title":title,"rom_bytes":len(rom),
                "rom_sha256":hashlib.sha256(rom).hexdigest(),
                "bios_sha256":hashlib.sha256(bios).hexdigest(),
                "expected_rom_crc32":f"{zlib.crc32(rom):08x}",
                "expected_bios_crc32":f"{zlib.crc32(bios):08x}",
                "physical_rom_crc32":match[2] if match else None,
                "physical_bios_crc32":bios_match[1] if bios_match else None,
                "fifo_high_water":int(match[3]) if match else None,
                "sdram_phase_ps":int(match[4]) if match else None,
                "memory_period_ps":int(match[5]) if match else None,
                "status":"pass" if passed else "fail","exit_code":completed.returncode,
                "elapsed_seconds":round(elapsed,3),"log":log,
            }

        cases=list(images(sizes))
        results=[]
        report_path=Path(os.environ.get("STORAGE_REPORT",ROOT/"build/sim/storage-integration.json"))
        report_path.parent.mkdir(parents=True,exist_ok=True)
        with ThreadPoolExecutor(max_workers=jobs) as pool:
            pending=[pool.submit(run_case,case) for case in enumerate(cases)]
            for future in as_completed(pending):
                result=future.result();results.append(result)
                print(f"\n{result['status'].upper()}: {result['title']} ({result['elapsed_seconds']:.1f}s)\n{result['log']}",flush=True)
                report={"recorded_utc":datetime.now(timezone.utc).isoformat(),"simulator":version,
                        "source_sha256":source_hashes,
                        "jobs":jobs,"expected_images":len(cases),"completed_images":len(results),
                        "corpus":"private" if os.environ.get("ROMSET") else "synthetic",
                        "bridge_clocks_per_word":75,"results":sorted(results,key=lambda row:row["index"])}
                report_path.write_text(json.dumps(report,indent=2)+"\n")
        failures=[row for row in results if row["status"]!="pass"]
        if failures:
            raise RuntimeError(f"{len(failures)} storage image(s) failed; see {report_path}")
        print(f"PASS: full physical CRC and CPU probes for {len(results)} image(s); report {report_path}",flush=True)


if __name__=="__main__":
    main()
