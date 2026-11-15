#!/usr/bin/env python3
"""
test_history.py - Scans Real Traffic Fix autotest and proving ground test logs,
extracts historical test records, writes docs/test_history.json, and prints a summary table.
"""

import os
import sys
import glob
import re
import json
import datetime
from pathlib import Path

DEFAULT_SCRIPTS_DIR = r"C:/Users/death/Games/GTA/GTA San Andreas/scripts"
DEFAULT_OUTPUT_JSON = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "docs", "test_history.json"))


def parse_datetime(folder_name, folder_path, log_lines):
    """Extract standard ISO YYYY-MM-DD HH:MM:SS string."""
    # Pattern 1: YYYYMMDD_HHMMSS
    m = re.search(r"(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})", folder_name)
    if m:
        return f"{m.group(1)}-{m.group(2)}-{m.group(3)} {m.group(4)}:{m.group(5)}:{m.group(6)}"
    
    # Pattern 2: Header line 'started HH:MM:SS'
    started_time = None
    for line in log_lines[:6]:
        m_s = re.search(r"started\s+(\d{2}:\d{2}:\d{2})", line)
        if m_s:
            started_time = m_s.group(1)
            break
            
    try:
        mtime = datetime.datetime.fromtimestamp(os.path.getmtime(folder_path))
        date_str = mtime.strftime("%Y-%m-%d")
        if started_time:
            return f"{date_str} {started_time}"
        return mtime.strftime("%Y-%m-%d %H:%M:%S")
    except Exception:
        return None


def parse_autotest_log(folder_path):
    """Parse an autotest folder containing autotest.log."""
    folder_name = os.path.basename(folder_path)
    log_path = os.path.join(folder_path, "autotest.log")
    if not os.path.isfile(log_path):
        return None

    try:
        with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()
    except Exception as e:
        return {
            "folder": folder_name,
            "datetime": None,
            "kind": "autotest",
            "scenario": "unknown",
            "car_model": None,
            "driver_profile": None,
            "result": f"ERROR: {e}",
            "minutes": None,
            "km": None,
            "collisions": None,
            "off_road_count": 0,
            "test_name": None,
            "ab": None,
            "offset_rms": None,
            "offset_max": None,
            "skid_pct": None
        }

    dt_str = parse_datetime(folder_name, folder_path, lines)
    content = "".join(lines)

    # Off road count
    off_road_count = len(re.findall(r"^=== OFF ROAD #\d+", content, re.MULTILINE))

    # Car model
    car_model = None
    # 1. From collision line: model 411 TESTCAR
    m_mod = re.search(r"model\s+(\d{3})\s+TESTCAR", content)
    if m_mod:
        car_model = m_mod.group(1)
    # 2. From OFF ROAD line: MAX model 411 at ...
    if not car_model:
        m_mod = re.search(r"=== OFF ROAD\s+#\d+[^:]*:\s*\w+\s+model\s+(\d{3})", content)
        if m_mod:
            car_model = m_mod.group(1)
    # 3. Model 411: mass
    if not car_model:
        m_mod = re.search(r"Model\s+(\d{3}):\s+mass", content)
        if m_mod:
            car_model = m_mod.group(1)
    # 4. Folder name inference
    if not car_model:
        fn_lower = folder_name.lower()
        if "infernus" in fn_lower:
            car_model = "411"
        elif "bullet" in fn_lower:
            car_model = "541"
        elif "sultan" in fn_lower:
            car_model = "560"
        elif "sandking" in fn_lower:
            car_model = "495"
        elif "cement" in fn_lower:
            car_model = "524"
        elif "premier" in fn_lower:
            car_model = "426"

    # Driver profile
    driver_profile = None
    # Check arrival or fail line
    arr_lines = [l.strip() for l in lines if "ARRIVED" in l or "FAIL" in l]
    for al in reversed(arr_lines):
        m_dr = re.search(r"\[(NORMAL|FAST|MAX|CALM|AGGRESSIVE|SOZINHO|ALONE|RACER|PLAYER|FLEEING|WHEELSPIN)\]", al, re.I)
        if m_dr:
            driver_profile = m_dr.group(1).upper()
            break
        m_dr2 = re.match(r"^(NORMAL|FAST|MAX|CALM|AGGRESSIVE|SOZINHO|ALONE|RACER|PLAYER|FLEEING|WHEELSPIN)\s+", al, re.I)
        if m_dr2:
            driver_profile = m_dr2.group(1).upper()
            break
    if not driver_profile:
        for l in reversed(lines):
            m_tr = re.search(r"\[trip\]\s+>(\w+):", l)
            if m_tr:
                driver_profile = m_tr.group(1).upper()
                break
    if not driver_profile:
        m_dr3 = re.search(r"route trip to [^:]*:\s*(\w+)\s+and\s+(\w+)\s+drivers", content)
        if m_dr3:
            driver_profile = m_dr3.group(1).upper()
    if not driver_profile:
        if "alone" in folder_name.lower() or "sozinho" in folder_name.lower():
            driver_profile = "ALONE"
        elif "max" in folder_name.lower():
            driver_profile = "MAX"

    # Scenario
    scenario = "Unknown"
    if "Grove Street -> Bayside" in content or "autotest_25_trip" in folder_name or "trip" in folder_name:
        scenario = "Grove Street -> Bayside"
    elif "Bayside -> LS Downtown" in content:
        scenario = "Bayside -> LS Downtown"
    elif "Freeway" in content or "hwy" in folder_name:
        scenario = "Freeway / Highway"
    elif "scan" in content or "scan" in folder_name:
        scenario = "Road Scan"
    elif "Location" in content:
        m_loc = re.findall(r"--- Location \d+/\d+:\s*([^\n\r]+?)\s+at\s+[\d\.]+\s+min", content)
        if m_loc:
            scenario = f"Locations ({len(m_loc)}): {m_loc[0]}"
        else:
            scenario = "Multi-location"
    elif "smoke" in folder_name.lower():
        scenario = "Smoke Test"
    elif "route_diag" in folder_name.lower():
        scenario = "Route Diag"

    # Result, minutes, km, collisions
    result = "UNKNOWN"
    minutes = None
    km = None
    collisions = None
    avg_speed_kmh = None

    # Check ARRIVED lines
    for al in reversed(lines):
        if "ARRIVED" in al:
            result = "ARRIVED"
            m_m = re.search(r"(?:after|at)\s+([\d\.]+)\s+min", al)
            if m_m:
                minutes = float(m_m.group(1))
            m_k = re.search(r"([\d\.]+)\s+km\s+driven", al)
            if m_k:
                km = float(m_k.group(1))
            m_c = re.search(r"(\d+)\s+collisions", al)
            if m_c:
                collisions = int(m_c.group(1))
            m_spd = re.search(r"average\s+(\d+)\s+km/h", al)
            if m_spd:
                avg_speed_kmh = float(m_spd.group(1))
            break

    # Check FAIL lines
    if result == "UNKNOWN":
        for al in reversed(lines):
            if "FAIL" in al:
                result = al.strip()
                m_m = re.search(r"(?:after|at)\s+([\d\.]+)\s+min", al)
                if m_m:
                    minutes = float(m_m.group(1))
                m_k = re.search(r"([\d\.]+)\s+km\s+driven", al)
                if m_k:
                    km = float(m_k.group(1))
                m_c = re.search(r"(\d+)\s+collisions", al)
                if m_c:
                    collisions = int(m_c.group(1))
                m_spd = re.search(r"average\s+(\d+)\s+km/h", al)
                if m_spd:
                    avg_speed_kmh = float(m_spd.group(1))
                break

    # Summary block parsing
    m_sum = re.search(
        r"mode\s+\w+,\s+([\d\.]+)\s+min\s+of\s+game\s+time(?:,\s+\d+\s+locations)?(?:,\s+test car drove\s+([\d\.]+)\s+km)?",
        content
    )
    if m_sum:
        if minutes is None:
            minutes = float(m_sum.group(1))
        if km is None and m_sum.group(2):
            km = float(m_sum.group(2))
        if result == "UNKNOWN":
            result = "COMPLETED"

    # Vehicle collisions in summary
    m_col = re.search(r"collisions between vehicles:\s*(\d+)", content)
    if m_col and collisions is None:
        collisions = int(m_col.group(1))

    # Oscillations / Jams
    oscillations = None
    m_osc = re.search(r"oscillations\s*\(forward/back/forward within 8 s\):\s*(\d+)", content)
    if m_osc:
        oscillations = int(m_osc.group(1))

    jams = None
    m_jam = re.search(r"jams\s*\(a mod-driven vehicle stopped > 60 s[^\)]*\):\s*(\d+)", content)
    if m_jam:
        jams = int(m_jam.group(1))

    # Partial / Incomplete run fallback
    if minutes is None:
        for l in reversed(lines):
            m_p = re.search(r"\[pulse\s+([\d\.]+)\s+min\]", l)
            if m_p:
                minutes = float(m_p.group(1))
                break
        if minutes is None:
            for l in reversed(lines):
                m_o = re.search(r"t=([\d\.]+)\s+min", l)
                if m_o:
                    minutes = float(m_o.group(1))
                    break

    if km is None or collisions is None:
        for l in reversed(lines):
            m_tr = re.search(r"\[trip\]\s+>[^:]*:\s*[\d\.]+\s+m\s+left,\s+[\d\.]+\s+km/h[^,]*,\s+([\d\.]+)\s+km,\s+(\d+)\s+coll", l)
            if m_tr:
                if km is None:
                    km = float(m_tr.group(1))
                if collisions is None:
                    collisions = int(m_tr.group(2))
                break

    if result == "UNKNOWN":
        if any("finished" in l for l in lines[-6:]):
            result = "COMPLETED"
        else:
            result = "DID NOT FINISH"

    return {
        "folder": folder_name,
        "datetime": dt_str,
        "kind": "autotest",
        "scenario": scenario,
        "car_model": car_model,
        "driver_profile": driver_profile,
        "result": result,
        "minutes": minutes,
        "km": km,
        "collisions": collisions,
        "off_road_count": off_road_count,
        "avg_speed_kmh": avg_speed_kmh,
        "oscillations": oscillations,
        "jams": jams,
        "test_name": None,
        "ab": None,
        "offset_rms": None,
        "offset_max": None,
        "skid_pct": None
    }


def parse_proving_summary(summary_path):
    """Parse a proving ground summary.txt file, yielding one record per test run."""
    folder_path = os.path.dirname(summary_path)
    folder_name = os.path.basename(folder_path)

    try:
        with open(summary_path, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()
    except Exception as e:
        return [{
            "folder": folder_name,
            "datetime": None,
            "kind": "proving",
            "scenario": "proving_ground",
            "car_model": None,
            "driver_profile": None,
            "result": f"ERROR: {e}",
            "minutes": None,
            "km": None,
            "collisions": 0,
            "off_road_count": 0,
            "test_name": None,
            "ab": None,
            "offset_rms": None,
            "offset_max": None,
            "skid_pct": None
        }]

    dt_str = parse_datetime(folder_name, folder_path, lines)

    # Summary header info: driver, steer, A/B parameters
    header_driver = "unknown"
    header_model = None
    for l in lines[:10]:
        m_dr = re.search(r"driver\s+([a-zA-Z0-9_\-]+)", l)
        if m_dr:
            header_driver = m_dr.group(1)
        m_mod = re.search(r"Model\s+(\d{3}):\s+mass", l)
        if m_mod:
            header_model = m_mod.group(1)

    records = []
    # Test line pattern:
    # [01A] slalom_fast model 411 400 km/h course 687 m end offset mean 0.09 rms 0.13 max 0.41 m | ...
    test_line_re = re.compile(
        r"^\[(\d+)([AB])?\]\s+(\w+)\s+model\s+(\d+)\s+(\d+)\s+km/h\s+(?:course\s+(\d+)\s+m\s+)?(\w+)\s+"
        r"offset\s+mean\s+([\d\.\-]+)\s+rms\s+([\d\.\-]+)\s+max\s+([\d\.\-]+)\s*m?"
    )

    for line in lines:
        line_clean = line.strip()
        if not line_clean.startswith("["):
            continue

        m = test_line_re.match(line_clean)
        if not m:
            continue

        test_num = m.group(1)
        ab_val = m.group(2) if m.group(2) else None
        test_name = m.group(3)
        car_model = m.group(4)
        target_spd = float(m.group(5))
        course_m = float(m.group(6)) if m.group(6) else None
        test_result = m.group(7)
        offset_mean = float(m.group(8))
        offset_rms = float(m.group(9))
        offset_max = float(m.group(10))

        # Additional metrics from line
        lat_g_max = None
        m_lat = re.search(r"lat\s+g\s+max\s+([\d\.\-]+)", line_clean)
        if m_lat:
            lat_g_max = float(m_lat.group(1))

        steer_mean_rad = None
        steer_rev = None
        m_str = re.search(r"steer\s+mean\s+([\d\.\-]+)\s+rad,\s+reversals\s+([\d\.\-]+)/s", line_clean)
        if m_str:
            steer_mean_rad = float(m_str.group(1))
            steer_rev = float(m_str.group(2))

        skid_front = None
        skid_rear = None
        m_skid = re.search(r"skid\s+front\s+(\d+)%\s+rear\s+(\d+)%", line_clean)
        if m_skid:
            skid_front = int(m_skid.group(1))
            skid_rear = int(m_skid.group(2))
        skid_str = f"front {skid_front}% rear {skid_rear}%" if skid_front is not None else None

        spd_err = None
        spd_max = None
        m_spe = re.search(r"speed\s+err\s+([\+\-\d\.]+)\s+km/h\s+max\s+([\d\.]+)", line_clean)
        if m_spe:
            spd_err = float(m_spe.group(1))
            spd_max = float(m_spe.group(2))

        km_val = round(course_m / 1000.0, 3) if course_m is not None else None

        records.append({
            "folder": folder_name,
            "datetime": dt_str,
            "kind": "proving",
            "scenario": "proving_ground",
            "car_model": car_model,
            "driver_profile": header_driver,
            "result": test_result,
            "minutes": None,
            "km": km_val,
            "collisions": 0,
            "off_road_count": 0,
            "test_name": test_name,
            "ab": ab_val,
            "offset_rms": offset_rms,
            "offset_max": offset_max,
            "skid_pct": skid_str,
            "skid_front_pct": skid_front,
            "skid_rear_pct": skid_rear,
            "offset_mean": offset_mean,
            "lat_g_max": lat_g_max,
            "target_speed_kmh": target_spd,
            "speed_max_kmh": spd_max,
            "speed_err_kmh": spd_err,
            "steer_mean_rad": steer_mean_rad,
            "steer_reversals_per_s": steer_rev,
        })

    # If the summary had 0 tests (e.g. no ground / failed setup)
    if not records:
        fail_msg = "FAILED_SETUP"
        for l in lines:
            if "no ground" in l.lower() or "error" in l.lower():
                fail_msg = l.strip()
                break
        records.append({
            "folder": folder_name,
            "datetime": dt_str,
            "kind": "proving",
            "scenario": "proving_ground",
            "car_model": header_model,
            "driver_profile": header_driver,
            "result": fail_msg,
            "minutes": None,
            "km": None,
            "collisions": 0,
            "off_road_count": 0,
            "test_name": None,
            "ab": None,
            "offset_rms": None,
            "offset_max": None,
            "skid_pct": None
        })

    return records


def scan_all_history(scripts_dir=DEFAULT_SCRIPTS_DIR, output_json=DEFAULT_OUTPUT_JSON):
    """Scans all autotest_* and proving/*/summary.txt in scripts_dir and writes output_json."""
    all_records = []

    # 1. Scan autotest_* folders
    at_pattern = os.path.join(scripts_dir, "autotest_*")
    at_folders = sorted(glob.glob(at_pattern))
    for folder in at_folders:
        rec = parse_autotest_log(folder)
        if rec is not None:
            all_records.append(rec)

    # 2. Scan proving ground summaries
    prov_pattern = os.path.join(scripts_dir, "proving", "*", "summary.txt")
    prov_files = sorted(glob.glob(prov_pattern))
    for pfile in prov_files:
        precs = parse_proving_summary(pfile)
        all_records.extend(precs)

    # Sort all records by datetime
    all_records.sort(key=lambda r: (r.get("datetime") or "", r.get("folder") or ""))

    # Write output JSON
    os.makedirs(os.path.dirname(output_json), exist_ok=True)
    with open(output_json, "w", encoding="utf-8") as f:
        json.dump(all_records, f, indent=2, ensure_ascii=False)

    return all_records


def print_summary_table(records):
    """Print a short human-readable table of recent and significant test runs."""
    print("=" * 110)
    print(f"{'DateTime':19} | {'Kind':8} | {'Scenario / Test':22} | {'Car':4} | {'Driver':9} | {'Result':12} | {'Min':5} | {'Km':6} | {'Coll':4} | {'OffR':4}")
    print("=" * 110)

    # Select representative records:
    # Key Grove->Bayside arrivals, key fails, and recent proving ground tests
    selected = []
    
    # 1. Grove -> Bayside ARRIVED runs
    bayside_arrived = [r for r in records if r["kind"] == "autotest" and r.get("scenario") == "Grove Street -> Bayside" and r.get("result") == "ARRIVED"]
    # Sample every Nth to keep table concise + include the latest 5
    if len(bayside_arrived) > 12:
        step = max(1, len(bayside_arrived) // 8)
        selected.extend(bayside_arrived[::step])
        selected.extend(bayside_arrived[-5:])
    else:
        selected.extend(bayside_arrived)

    # 2. Recent proving ground runs
    proving_runs = [r for r in records if r["kind"] == "proving" and r.get("test_name")]
    if proving_runs:
        # take latest session's runs
        latest_pf = proving_runs[-1]["folder"]
        selected.extend([r for r in proving_runs if r["folder"] == latest_pf][:8])

    # Deduplicate and sort
    seen = set()
    dedup = []
    for r in selected:
        key = (r["datetime"], r["folder"], r.get("test_name"), r.get("ab"))
        if key not in seen:
            seen.add(key)
            dedup.append(r)
    dedup.sort(key=lambda r: (r.get("datetime") or "", r.get("folder") or ""))

    for r in dedup[-25:]:
        dt = r.get("datetime") or "Unknown"
        kind = r.get("kind") or ""
        scen = (r.get("test_name") + (f" [{r.get('ab')}]" if r.get('ab') else "")) if r.get("kind") == "proving" else (r.get("scenario") or "")[:22]
        car = r.get("car_model") or "-"
        dr = (r.get("driver_profile") or "-")[:9]
        res = (r.get("result") or "")[:12]
        mins = f"{r['minutes']:.2f}" if r.get("minutes") is not None else "-"
        kms = f"{r['km']:.2f}" if r.get("km") is not None else "-"
        coll = str(r["collisions"]) if r.get("collisions") is not None else "-"
        offr = str(r["off_road_count"]) if r.get("off_road_count") is not None else "-"

        print(f"{dt:19} | {kind:8} | {scen:22} | {car:4} | {dr:9} | {res:12} | {mins:>5} | {kms:>6} | {coll:>4} | {offr:>4}")
    print("=" * 110)
    print(f"Total historical records: {len(records)} (Autotests: {sum(1 for r in records if r['kind']=='autotest')}, Proving: {sum(1 for r in records if r['kind']=='proving')})")


def main():
    scripts_dir = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SCRIPTS_DIR
    output_json = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUTPUT_JSON

    print(f"Scanning test logs from: {scripts_dir}")
    print(f"Writing output JSON to: {output_json}")

    records = scan_all_history(scripts_dir, output_json)
    print_summary_table(records)
    print(f"\nSuccessfully wrote {len(records)} test records to {output_json}")


if __name__ == "__main__":
    main()
