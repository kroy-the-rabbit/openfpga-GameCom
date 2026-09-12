#!/usr/bin/env python3
"""Asset contract and fail-closed build gate regression tests."""
import importlib.util
import json
from pathlib import Path
import tempfile
ROOT=Path(__file__).resolve().parents[2]
core=ROOT/"pkg/Cores/kroy.GameCom"
meta=json.loads((core/"core.json").read_text())["core"]
assert meta["framework"]["hardware"]["cartridge_adapter"]=="0x80000000"
assert meta["framework"]["sleep_supported"] is False
slots=json.loads((core/"data.json").read_text())["data"]["data_slots"]
assert [s["id"] for s in slots]==[1,4]
assert [s["address"] for s in slots]==["0x10000000","0x30000000"]
assert slots[1]["size_exact"]==0x40000 and int(slots[0]["size_maximum"],16)==0x200000
assert not list((ROOT/"pkg").rglob("*.tgc")) and not list((ROOT/"pkg").rglob("*.bin"))
spec=importlib.util.spec_from_file_location("report",ROOT/"scripts/report.py")
report=importlib.util.module_from_spec(spec);spec.loader.exec_module(report)
fit="Fitter Status : Successful - Fri Sep 11\n"
sta="".join(f"Type : Slow 1100mV 85C Model {k} 'clock'\nSlack : 0.100\n" for k in ("Setup","Hold","Recovery","Removal","Minimum Pulse Width"))
assert len(report.check(fit,sta))==5
for bad_fit,bad_sta in [("Failed",sta),(fit,""),(fit,sta.replace("0.100","-0.001",1)),(fit,sta.replace("0.100","N/A",1)),(fit,sta.replace("Model Hold","Model Unknown"))]:
    try: report.check(bad_fit,bad_sta)
    except ValueError: pass
    else: raise AssertionError("Build gate accepted missing/invalid timing")
print("PASS APF asset contract, no ROM payloads, build gate rejects missing/failing analyses")

ucp="""; Unconstrained Paths Summary ;
; Property ; Setup ; Hold ;
; Illegal Clocks ; 0 ; 0 ;
; Unconstrained Clocks ; 0 ; 0 ;
; Unconstrained Input Ports ; 0 ; 0 ;
; Unconstrained Input Port Paths ; 0 ; 0 ;
; Unconstrained Output Ports ; 0 ; 0 ;
; Unconstrained Output Port Paths ; 0 ; 0 ;
; Unconstrained Input Ports ;
"""
assert len(report.check_unconstrained(ucp))==6
for bad_ucp in ("", ucp.replace("; 0 ; 0 ;", "; 0 ; 1 ;", 1),
                ucp.replace("; 0 ; 0 ;", "; N/A ; 0 ;", 1),
                ucp.replace("; Unconstrained Input Port Paths ; 0 ; 0 ;", ""),
                ucp + "; Illegal Clocks ; 0 ; 0 ;\n",
                ucp.replace("; Unconstrained Input Ports ; 0 ; 0 ;", "; Unconstrained Input Ports ; 1 ; 1 ;")
                   .replace("; Unconstrained Input Port Paths ; 0 ; 0 ;", "; Unconstrained Input Port Paths ; 20 ; 20 ;")):
    try: report.check_unconstrained(bad_ucp)
    except ValueError: pass
    else: raise AssertionError("Build gate accepted missing/invalid/unconstrained timing paths")
print("PASS unconstrained-path gate: all six summaries required; missing, duplicate, invalid and nonzero rejected")

real_net="""; Net Delay ;
; Name ; Slack ; Required ; Actual ; From ; To ; Type ;
; set_net_delay ; 93.628 ; 100.000 ; 6.372 ; [get_registers {core_top:ic|gb:gb|gb_savestates:gb_savestates|reset_out}] ; [get_registers {core_top:ic|gb:gb|video:video|obpd[47][4]}] ; max ;
; -- ; 93.628 ; 100.000 ; 6.372 ; core_top:ic|gb:gb|gb_savestates:gb_savestates|reset_out ; core_top:ic|gb:gb|video:video|obpd[47][4] ; max ;
"""
assert len(report.net_delay_rows(real_net))==1
try: report.net_delay_rows("\n".join(line for line in real_net.splitlines() if not line.startswith("; -- ;")))
except ValueError: pass
else: raise AssertionError("CDC gate counted a constraint summary as reported edge coverage")
real_skew="""; Name ; Slack ; Required Skew ; Actual Skew ; From Node ; To Node ; Launch Clock ; Latch Clock ; Options ;
; set_max_skew ; 98.071 ; 100.000 ; 1.929 ; [get_registers {core_top:ic|gb:gb|gb_savestates:gb_savestates|reset_out}] ; [get_registers {core_top:ic|gb:gb|video:video|obpd[47][4]}] ; ; ; ;
; - Skew for the Latest Arrival ; ; ; ; ; ; ; ; ;
; -- ; 98.071 ; 100.000 ; 1.929 ; core_top:ic|gb:gb|gb_savestates:gb_savestates|reset_out ; core_top:ic|gb:gb|video:video|obpd[47][4] ; clk ; clk ; ;
; Path Summary ;
; Property ; Value ;
; From Node ; core_top:ic|gb:gb|gb_savestates:gb_savestates|reset_out ;
; Slack ; 98.071 ;
; Latest Path Arrival ;
; Total ; Incr ; RF ; Type ; Fanout ; Location ; Element ;
; 2.736 ; 0.000 ; FF ; CELL ; 2359 ; FF_X37_Y22_N53 ; ic|gb|gb_savestates|reset_out|q ;
"""
assert len(report.analyzed_path_rows(real_skew,"skew"))==1
try: report.analyzed_path_rows("\n".join(line for line in real_skew.splitlines() if not line.startswith("; -- ;")),"skew")
except ValueError: pass
else: raise AssertionError("CDC gate counted a skew constraint summary or detailed delay row as a path result")

with tempfile.TemporaryDirectory(prefix="gamecom-cdc-gate-") as temporary:
    directory=Path(temporary)
    fit_path=directory/"ap_core.fit.summary"
    fit_path.write_text(fit)
    stamp=fit_path.stat()
    base=["VERSION\t3",f"FIT\t{int(stamp.st_mtime)}\t{stamp.st_size}"]
    net=["; Slack ; Required ; Actual ; From Node ; To Node ;"]
    skew_names=[]
    path_texts={}
    for bus,(source,destination,minimum) in report.CDC_BUSES.items():
        base.append(f"ENDPOINT\t{bus}\t{minimum}\t{minimum}")
        path_lines=["; Slack ; From Node ; To Node ; Launch Clock ; Latch Clock ; Relationship ; Clock Skew ; Data Delay ;"]
        for bit in range(minimum):
            base += [f"NODE\t{bus}\tfrom\tic|loader|{source}[{bit}]",
                     f"NODE\t{bus}\tto\tic|loader|{destination}[{bit}]",
                     f"CANDIDATE\t{bus}\tic|loader|{destination}[{bit}]",
                     f"FANOUT\t{bus}\tic|loader|{destination}[{bit}]"]
            if bus in report.NET_BUSES:
                net.append(f"; 1.250 ; {report.PATH_BUDGETS[bus]:.3f} ; {report.PATH_BUDGETS[bus]-1.25:.3f} ; ic|loader|{source}[{bit}] ; ic|loader|{destination}[{bit}] ;")
            path_lines.append(f"; 1.000 ; ic|loader|{source}[{bit}] ; ic|loader|{destination}[{bit}] ; launch ; latch ; {report.PATH_BUDGETS[bus]:.3f} ; 0.000 ; 1.000 ;")
        path_texts[bus]="\n".join(path_lines)+"\n"
        if bus in report.GRAY_BUSES:
            source_filter,destination_filter=report.GRAY_FILTERS[bus]
            skew_names.append(f"; set_max_skew ; 1.500 ; 10.000 ; 8.500 ; [get_registers {{{source_filter}}}] ; [get_registers {{{destination_filter}}}] ;")
            skew_names.append(f"; -- ; 1.500 ; 10.000 ; 8.500 ; ic|loader|{source}[0] ; ic|loader|{destination}[0] ;")
    for corner in range(2):
        base += [f"CORNER\t{corner}\tcorner-{corner}",f"SKEW\t{corner}\t6\t1.500"]
    base.append("COMPLETE\t2")
    evidence="\n".join(base)+"\n"
    net_text="\n".join(net)+"\n"
    skew_text="; Name ; Slack ; Required Skew ; Actual Skew ; From Node ; To Node ;\n"+"\n".join(skew_names)+"\n"
    def fixture():
        (directory/"cdc-evidence.tsv").write_text(evidence)
        for corner in range(2):
            (directory/f"net-delay-{corner}.txt").write_text(net_text)
            (directory/f"max-skew-{corner}.txt").write_text(skew_text)
            for bus, paths in path_texts.items():
                (directory/f"cdc-path-{bus}-{corner}.txt").write_text(paths)
    def reject(mutation):
        fixture()
        mutation()
        try: report.check_cdc(directory,fit_path)
        except (ValueError,OSError): pass
        else: raise AssertionError("CDC gate accepted absent, stale, or failing evidence")
    fixture()
    assert len(report.check_cdc(directory,fit_path))==2*(len(report.CDC_BUSES)+len(report.NET_BUSES)+1)
    reject(lambda: (directory/"cdc-path-rtc_payload-1.txt").unlink())
    reject(lambda: (directory/"cdc-path-rtc_payload-0.txt").write_text("No paths found\n"))
    reject(lambda: (directory/"cdc-path-rtc_payload-0.txt").write_text(path_texts["rtc_payload"].replace("50.000", "100.000")))
    reject(lambda: (directory/"cdc-path-slot_size-0.txt").write_text(path_texts["slot_size"].replace("; 1.000 ;", "; -0.001 ;",1)))
    reject(lambda: (directory/"cdc-path-fifo_write-0.txt").write_text("\n".join(line for line in path_texts["fifo_write"].splitlines() if "|wr_gray_rd1[1]" not in line)))
    reject(lambda: (directory/"cdc-evidence.tsv").unlink())
    reject(lambda: (directory/"net-delay-1.txt").unlink())
    reject(lambda: (directory/"max-skew-1.txt").write_text("No paths found\n"))
    reject(lambda: (directory/"cdc-evidence.tsv").write_text(evidence.replace("COMPLETE\t2\n","")))
    reject(lambda: (directory/"cdc-evidence.tsv").write_text(evidence.replace(f"FIT\t{int(stamp.st_mtime)}", "FIT\t0")))
    reject(lambda: (directory/"cdc-evidence.tsv").write_text(evidence.replace("SKEW\t1\t6\t1.500", "SKEW\t1\t6\t-0.010")))
    reject(lambda: (directory/"cdc-evidence.tsv").write_text(evidence.replace("ENDPOINT\tfifo_write\t2\t2", "ENDPOINT\tfifo_write\t0\t2")))
    reject(lambda: (directory/"cdc-evidence.tsv").write_text(evidence.replace("CORNER\t1\tcorner-1\n","")))
    reject(lambda: (directory/"net-delay-0.txt").write_text(net_text.replace("1.250", "-0.001", 1)))
    reject(lambda: (directory/"net-delay-0.txt").write_text(net_text.replace("1.250", "N/A", 1)))
    reject(lambda: (directory/"net-delay-0.txt").write_text(net_text.replace("1.250", "NaN", 1)))
    reject(lambda: (directory/"net-delay-0.txt").write_text(net_text.replace("|wr_gray[", "|unmatched[")))
    reject(lambda: (directory/"net-delay-0.txt").write_text(net_text.replace("|announced_bytes[", "|expected_size[")))
    reject(lambda: (directory/"net-delay-0.txt").write_text("\n".join(line for line in net_text.splitlines() if "|wr_gray_rd1[1]" not in line)))
    reject(lambda: (directory/"net-delay-0.txt").write_text(net_text.replace("[0]", "[*]").replace("[1]", "[*]")))
    reject(lambda: (directory/"max-skew-0.txt").write_text(skew_text.replace("10.000", "100.000")))
    reject(lambda: (directory/"max-skew-0.txt").write_text(skew_text.replace("Required Skew", "Missing Requirement")))
    reject(lambda: (directory/"max-skew-0.txt").write_text(skew_text.replace("10.000", "NaN")))
    reject(lambda: (directory/"max-skew-0.txt").write_text(skew_text.replace("wr_gray[*]", "wr_gray[0]")))
    reject(lambda: (directory/"max-skew-0.txt").write_text(skew_text.replace("wr_gray_rd1[*]", "wr_gray_rd1[0]")))
    reject(lambda: (directory/"max-skew-0.txt").write_text("\n".join(
        line for line in skew_text.splitlines() if not line.startswith("; set_max_skew ;"))))
    reject(lambda: (directory/"net-delay-0.txt").write_text(net_text.replace("10.000", "100.000")))
    reject(lambda: (directory/"net-delay-0.txt").write_text(net_text.replace("Required", "Missing Requirement")))
    reject(lambda: (directory/"cdc-path-fifo_write-0.txt").write_text(
        path_texts["fifo_write"].replace("ic|loader|wr_gray[", "ic|unrelated|wr_gray[")))

    def duplicated_destination(include_original):
        original="ic|loader|wr_gray_rd1[0]"
        duplicate=original+"~DUPLICATE"
        changed=evidence.replace("ENDPOINT\tfifo_write\t2\t2", "ENDPOINT\tfifo_write\t2\t3")
        changed=changed.replace("CORNER\t0", f"NODE\tfifo_write\tto\t{duplicate}\nCANDIDATE\tfifo_write\t{duplicate}\nFANOUT\tfifo_write\t{duplicate}\nCORNER\t0",1)
        (directory/"cdc-evidence.tsv").write_text(changed)
        for corner in range(2):
            for filename in (f"net-delay-{corner}.txt",f"cdc-path-fifo_write-{corner}.txt"):
                target=directory/filename
                original_text=target.read_text()
                if include_original:
                    duplicate_rows=[line.replace(original,duplicate) for line in original_text.splitlines() if original in line]
                    target.write_text(original_text+"\n".join(duplicate_rows)+"\n")
                else:
                    target.write_text(original_text.replace(original,duplicate))
    fixture()
    duplicated_destination(include_original=True)
    report.check_cdc(directory,fit_path)
    fixture()
    unrelated="ic|loader|error[0]_OTERM_OTHER_CAUSE"
    extra_candidate=f"CANDIDATE\tcrc_bad_error\t{unrelated}\n"
    (directory/"cdc-evidence.tsv").write_text(evidence.replace("CORNER\t0",extra_candidate+"CORNER\t0",1))
    report.check_cdc(directory,fit_path)
    reject(lambda: (directory/"cdc-evidence.tsv").write_text(evidence.replace(
        "CORNER\t0",extra_candidate+f"FANOUT\tcrc_bad_error\t{unrelated}\nCORNER\t0",1)))
    reject(lambda: (directory/"cdc-evidence.tsv").write_text("\n".join(
        line for line in evidence.splitlines() if not line.startswith("FANOUT\tcrc_bad_error\t"))))
    reject(lambda: (directory/"cdc-evidence.tsv").write_text("\n".join(
        line for line in evidence.splitlines() if not line.startswith("CANDIDATE\tcrc_bad_error\t"))))
    fixture()
    for corner in range(2):
        target=directory/f"max-skew-{corner}.txt"
        target.write_text(target.read_text().replace("[get_registers {", "[get_registers    {"))
    report.check_cdc(directory,fit_path)
    reject(lambda: duplicated_destination(include_original=False))

    fixture()
    for filename in ("cdc-evidence.tsv","cdc-path-slot_kind-0.txt","cdc-path-slot_kind-1.txt"):
        target=directory/filename
        target.write_text(target.read_text().replace("active_bios[0]","active_bios~DUPLICATE")
                          .replace("mem_bios[0]","mem_bios"))
    report.check_cdc(directory,fit_path)
assert report.datapath_rows("; Slack ; From Node ; To Node ; Relationship ;\n; 1.000 ; ic|loader|active_bios ; ic|loader|mem_bios ; 20.000 ;\n",20.0)
print("PASS CDC gate: per-corner full-path/net/skew evidence, exact budgets, all bits, missing/stale/negative/non-numeric/unmatched rejection")
