#!/usr/bin/env python3
"""Fail closed on missing timing analyses, unsuccessful fit, or negative slack."""
import os
import math
from pathlib import Path
import re
import sys
from typing import NamedTuple

CDC_BUSES = {
    "fifo_write": ("wr_gray", "wr_gray_rd1", 2),
    "fifo_read": ("rd_gray", "rd_gray_wr1", 2),
    "committed": ("committed_gray", "committed_gray_b1", 2),
    "slot_size": ("announced_bytes", "mem_size", 1),
    "slot_kind": ("active_bios", "mem_bios", 1),
    "backing_base": ("backing_base", "mem_base", 1),
    "expected_crc": ("input_crc", "expected_crc_m1", 1),
    "result_crc": ("mem_result_crc", "readback_crc32", 1),
    "runtime_size": ("rom_size", "rom_size_s", 1),
    "crc_bad_bios": ("mem_crc_bad", "bios_loaded", 1),
    "crc_bad_cart": ("mem_crc_bad", "cart_loaded", 1),
    "crc_bad_error": ("mem_crc_bad", "error", 1),
    "rtc_payload": ("mailbox_data", "host_time", 1),
    "audio_payload": ("held_sample", "next_sample", 1),
}
GRAY_BUSES = {"fifo_write", "fifo_read", "committed"}
GRAY_FILTERS = {
    bus: tuple(("*loader|transfer_fifo|" if bus.startswith("fifo_") else "*loader|") + name + "[*]"
               for name in CDC_BUSES[bus][:2])
    for bus in GRAY_BUSES
}
PATH_BUDGETS = {bus: (10.0 if bus in GRAY_BUSES else
                      50.0 if bus in {"rtc_payload", "audio_payload"} else 20.0)
                for bus in CDC_BUSES}
NET_BUSES = set(CDC_BUSES) - {"rtc_payload", "slot_kind", "crc_bad_bios", "crc_bad_cart", "crc_bad_error"}


class TimingPath(NamedTuple):
    slack: float
    source: str
    destination: str
    required: float | None


def numeric_slack(value, description):
    try:
        number = float(value)
    except ValueError as exc:
        raise ValueError(f"Non-numeric {description}: {value}") from exc
    if not math.isfinite(number) or number < 0:
        raise ValueError(f"Failing {description}: {value}")
    return number


def check_unconstrained(text):
    """Require the complete Quartus UCP summary and zero unbounded IO/clocks."""
    required = {
        "Illegal Clocks", "Unconstrained Clocks", "Unconstrained Input Ports",
        "Unconstrained Input Port Paths", "Unconstrained Output Ports",
        "Unconstrained Output Port Paths",
    }
    found = {}
    for line in text.splitlines():
        if not line.lstrip().startswith(";"):
            continue
        cells = [cell.strip() for cell in line.strip().strip(";").split(";")]
        if not cells or cells[0] not in required:
            continue
        if len(cells) == 1:
            continue
        name = cells[0]
        if name in found or len(cells) != 3 or any(not re.fullmatch(r"\d+", value) for value in cells[1:]):
            raise ValueError(f"Invalid unconstrained-path summary: {line}")
        found[name] = tuple(map(int, cells[1:]))
        if any(found[name]):
            raise ValueError(f"Unconstrained timing: {name}, setup/hold={found[name]}")
    if set(found) != required:
        raise ValueError("Missing required unconstrained-path summary rows")
    return found


def analyzed_path_rows(text, description, row_name="--"):
    """Read actual paths from Quartus ASCII net-delay/max-skew tables."""
    slack_column = None
    name_column = None
    source_column = destination_column = required_column = None
    rows = []
    for line in text.splitlines():
        if not line.lstrip().startswith(";"):
            continue
        cells = [cell.strip() for cell in line.strip().strip(";").split(";")]
        headings = [i for i, cell in enumerate(cells) if re.search(r"\bslack\b", cell, re.I)]
        node_headings = {cell.lower() for cell in cells}
        if headings and node_headings & {"from", "from node"} and node_headings & {"to", "to node"}:
            slack_column = headings[0]
            name_column = next((i for i, cell in enumerate(cells) if cell.lower() == "name"), None)
            source_column = next(i for i, cell in enumerate(cells) if cell.lower() in {"from", "from node"})
            destination_column = next(i for i, cell in enumerate(cells) if cell.lower() in {"to", "to node"})
            required_column = next((i for i, cell in enumerate(cells) if cell.lower() in {"required", "required skew"}), None)
            continue
        if slack_column is None or len(cells) <= slack_column:
            continue
        if row_name != "--" and name_column is None:
            continue
        if name_column is not None and (len(cells) <= name_column or cells[name_column] != row_name):
            continue
        raw = cells[slack_column]
        is_number = bool(re.fullmatch(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?", raw))
        has_node = any("|" in cell for cell in cells)
        if is_number or has_node or raw.lower() in {"nan", "inf", "-inf", "n/a", "--"}:
            needed = [source_column, destination_column]
            if required_column is not None:
                needed.append(required_column)
            if len(cells) <= max(needed):
                raise ValueError(f"Incomplete {description} row: {line}")
            required = (numeric_slack(cells[required_column], f"{description} requirement")
                        if required_column is not None else None)
            rows.append(TimingPath(numeric_slack(raw, description), cells[source_column],
                                   cells[destination_column], required))
    if not rows:
        raise ValueError(f"Missing numeric {description} path results")
    return rows


def net_delay_rows(text):
    return analyzed_path_rows(text, "CDC net-delay slack")


def datapath_rows(text, budget):
    """Read the timing summary, checking the actual max-delay relationship."""
    columns, rows = None, []
    for line in text.splitlines():
        if re.match(r"\s*Path #\d+:", line):
            break  # Detailed arrival/required tables are not additional paths.
        if not line.lstrip().startswith(";"):
            continue
        cells = [cell.strip() for cell in line.strip().strip(";").split(";")]
        headings = {cell.lower(): index for index, cell in enumerate(cells)}
        if {"slack", "from node", "to node", "relationship"} <= set(headings):
            columns = headings
            continue
        if columns is None or len(cells) <= max(columns.values()):
            continue
        raw = cells[columns["slack"]]
        value = numeric_slack(raw, "CDC datapath setup slack")
        relationship = numeric_slack(cells[columns["relationship"]], "CDC datapath relationship")
        if not math.isclose(relationship, budget, abs_tol=0.001):
            raise ValueError(f"Missing CDC max-delay budget: expected {budget}, reported {relationship}")
        rows.append(TimingPath(value, cells[columns["from node"]], cells[columns["to node"]], relationship))
    if not rows:
        raise ValueError("Missing numeric CDC datapath timing results")
    return rows


def check_cdc(directory, fit_path):
    """Require implemented endpoints and passing reports for every fit corner."""
    records = [line.split("\t") for line in (directory / "cdc-evidence.tsv").read_text().splitlines()]
    endpoints, nodes, corners, skew = {}, {}, {}, {}
    candidates, fanouts = {}, {}
    version, fit_stamp, complete = None, None, None
    for record in records:
        if not record:
            continue
        kind, *values = record
        try:
            if kind == "VERSION" and len(values) == 1 and version is None:
                version = int(values[0])
            elif kind == "FIT" and len(values) == 2 and fit_stamp is None:
                fit_stamp = tuple(map(int, values))
            elif kind == "ENDPOINT" and len(values) == 3 and values[0] not in endpoints:
                endpoints[values[0]] = tuple(map(int, values[1:]))
            elif kind == "NODE" and len(values) == 3 and values[1] in {"from", "to"}:
                nodes.setdefault((values[0], values[1]), set()).add(values[2])
            elif kind in {"CANDIDATE", "FANOUT"} and len(values) == 2:
                collection = candidates if kind == "CANDIDATE" else fanouts
                actual = collection.setdefault(values[0], set())
                if values[1] in actual:
                    raise ValueError(f"Duplicate CDC connectivity record: {record}")
                actual.add(values[1])
            elif kind == "CORNER" and len(values) == 2 and values[0] not in corners:
                corners[values[0]] = values[1]
            elif kind == "SKEW" and len(values) == 3 and values[0] not in skew:
                skew[values[0]] = (int(values[1]), numeric_slack(values[2], "CDC max-skew slack"))
            elif kind == "COMPLETE" and len(values) == 1 and complete is None:
                complete = int(values[0])
            else:
                raise ValueError(f"Unexpected CDC evidence record: {record}")
        except (ValueError, IndexError) as exc:
            raise ValueError(f"Invalid CDC evidence record: {record}: {exc}") from exc
    stat = fit_path.stat()
    if version != 3 or fit_stamp != (int(stat.st_mtime), stat.st_size):
        raise ValueError("CDC evidence is missing, unsupported, or belongs to a different fit")
    if not corners or complete != len(corners) or set(corners) != set(skew):
        raise ValueError("CDC evidence has missing/incomplete operating corners")
    if set(corners) != {str(i) for i in range(complete)}:
        raise ValueError("CDC evidence has an invalid corner sequence")
    if set(endpoints) != set(CDC_BUSES):
        raise ValueError("CDC evidence is missing required crossing endpoints")
    if set(candidates) != set(CDC_BUSES) or set(fanouts) != set(CDC_BUSES):
        raise ValueError("CDC evidence is missing structural connectivity")
    for bus, (_, _, minimum) in CDC_BUSES.items():
        for direction, count in zip(("from", "to"), endpoints[bus]):
            actual = nodes.get((bus, direction), set())
            if count < minimum or len(actual) != count:
                raise ValueError(f"Missing implemented {bus} {direction} endpoints")
        reachable = candidates[bus] & fanouts[bus]
        if nodes[(bus, "to")] != reachable:
            raise ValueError(f"CDC destination coverage differs from structural fanouts for {bus}")
    results = {}
    for corner, name in corners.items():
        net_text = (directory / f"net-delay-{corner}.txt").read_text()
        rows = net_delay_rows(net_text)
        skew_text = (directory / f"max-skew-{corner}.txt").read_text()
        skew_rows = analyzed_path_rows(skew_text, "CDC max-skew slack")
        skew_constraints = analyzed_path_rows(skew_text, "CDC max-skew constraint", "set_max_skew")
        if skew[corner][0] < len(GRAY_BUSES):
            raise ValueError(f"Missing max-skew paths at {name}")
        for bus, (source, destination, _) in CDC_BUSES.items():
            source_nodes = nodes[(bus, "from")]
            destination_nodes = nodes[(bus, "to")]
            path_text = (directory / f"cdc-path-{bus}-{corner}.txt").read_text()
            analyses = {"path": datapath_rows(path_text, PATH_BUDGETS[bus])}
            if bus in NET_BUSES:
                analyses["net"] = rows
            for analysis, candidates in analyses.items():
                paths = [path for path in candidates if path.source in source_nodes
                         and path.destination in destination_nodes]
                if not paths:
                    raise ValueError(f"Missing analyzed {analysis} paths for {bus} at {name}")
                reported_destinations = {path.destination for path in paths}
                for destination_node in destination_nodes:
                    if destination_node not in reported_destinations:
                        raise ValueError(f"Missing analyzed {analysis} bit {destination_node} at {name}")
                if analysis == "net":
                    for path in paths:
                        if path.required is None or not math.isclose(path.required, PATH_BUDGETS[bus], abs_tol=0.001):
                            raise ValueError(f"Missing CDC net-delay budget for {bus}: expected {PATH_BUDGETS[bus]}, reported {path.required}")
                results[f"{name}/{bus}/{analysis}"] = min(path.slack for path in paths)
            if bus in GRAY_BUSES:
                source_filter, destination_filter = (
                    f"[get_registers {{{pattern}}}]" for pattern in GRAY_FILTERS[bus])
                constraints = [constraint for constraint in skew_constraints
                               if " ".join(constraint.source.split()) == source_filter
                               and " ".join(constraint.destination.split()) == destination_filter]
                if not constraints:
                    raise ValueError(f"Missing complete-vector max-skew constraint for {bus} at {name}")
                paths = [path for path in skew_rows if path.source in source_nodes
                         and path.destination in destination_nodes]
                if not paths:
                    raise ValueError(f"Missing analyzed max-skew paths for {bus} at {name}")
                for path in paths + constraints:
                    if path.required is None or not math.isclose(path.required, PATH_BUDGETS[bus], abs_tol=0.001):
                        raise ValueError(f"Missing CDC max-skew budget for {bus}: expected {PATH_BUDGETS[bus]}, reported {path.required}")
        results[f"{name}/gray/skew"] = skew[corner][1]
    return results

def check(fit, sta):
    if not re.search(r"Fitter Status\s*:\s*Successful", fit):
        raise ValueError("Fitter did not report success")
    slacks = {}
    kind = None
    for line in sta.splitlines():
        if line.startswith("Type"):
            m = re.search(r"Model (Setup|Hold|Recovery|Removal|Minimum Pulse Width)\b", line)
            kind = m.group(1) if m else None
        if line.startswith("Slack"):
            if kind is None:
                raise ValueError(f"Unrecognized timing analysis: {line}")
            m = re.search(r":\s*(-?\d+(?:\.\d+)?)", line)
            if not m:
                raise ValueError(f"Non-numeric timing slack: {line}")
            slacks[kind] = min(slacks.get(kind, float("inf")), float(m.group(1)))
    for kind in ("Setup", "Hold", "Minimum Pulse Width"):
        if kind not in slacks:
            raise ValueError(f"Missing required {kind} timing analysis")
    if min(slacks.values()) < 0:
        raise ValueError(f"Negative timing slack: {slacks}")
    return slacks

def main():
    bdir = Path(sys.argv[1])
    out = bdir / "work/src/fpga/build/output_files"
    marker = bdir / "TIMING_FAILED"
    try:
        fit = (out / "ap_core.fit.summary").read_text()
        sta = (out / "ap_core.sta.summary").read_text()
        slacks = check(fit, sta)
        check_unconstrained((bdir / "work/timing-paths/unconstrained.txt").read_text())
        cdc_slacks = check_cdc(bdir / "work/timing-paths", out / "ap_core.fit.summary")
    except (ValueError, OSError) as exc:
        marker.write_text(str(exc) + "\n")
        print(f"TIMING FAILED: {exc}", file=sys.stderr)
        return 3
    marker.unlink(missing_ok=True)
    report = ["core: kroy.GameCom", f"commit: {os.environ.get('GIT_SHA', 'unknown')}"]
    report += [f"{k:22s} {v:8.3f} ns" for k, v in slacks.items()]
    report += [f"CDC {k}: {v:.3f} ns" for k, v in cdc_slacks.items()]
    report += ["Unconstrained IO/clocks: 0 setup, 0 hold"]
    report += ["", fit, "", sta]
    (bdir / "report.txt").write_text("\n".join(report))
    print("\n".join(report[:3+len(slacks)+len(cdc_slacks)]))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
