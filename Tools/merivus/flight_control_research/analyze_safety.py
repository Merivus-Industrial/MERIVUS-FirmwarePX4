#!/usr/bin/env python3
"""Check AFCR zero-output boundaries against ULog mode and diagnostic samples."""

import argparse
import bisect
import json
import math
from pathlib import Path

from pyulog import ULog

import evaluate


EXPECTED_MODE = {
    "position_to_altitude": 1,
    "position_to_stabilized": 15,
    "stabilized_to_position": 2,
    "rtl": 5,
    "landing": 18,
    "takeoff": 17,
}
PRECURSOR_REQUIRED = {"manual_stick", "position_to_altitude", "position_to_stabilized",
                      "rtl", "failsafe", "gps_loss", "ekf_restart"}


def name_at(data, index):
    return bytes(int(data[f"name[{byte}]"][index]) for byte in range(10)).split(b"\0", 1)[0]


def score_boundary(ulog, event):
    log = ULog(str(ulog))
    status = evaluate.dataset(log, "vehicle_status")
    debug = evaluate.dataset(log, "debug_vect")
    start = int(event["start_s"] * 1e6)
    end = int(event["end_s"] * 1e6)
    modes = [int(status["nav_state"][i]) for i, stamp in enumerate(status["timestamp"])
             if start <= int(stamp) < end]
    mode_expected = EXPECTED_MODE.get(event["name"])
    corrections = []
    for i, stamp in enumerate(debug["timestamp"]):
        stamp = int(stamp)
        if not start <= stamp < end or name_at(debug, i) != b"AFCR_DA":
            continue
        status_index = bisect.bisect_right(status["timestamp"], stamp) - 1
        if status_index < 0:
            continue
        if mode_expected is not None and int(status["nav_state"][status_index]) != mode_expected:
            continue
        if event["name"] == "failsafe" and not bool(status["failsafe"][status_index]):
            continue
        grace_us = 500_000 if event["name"] in ("gps_loss", "failsafe", "ekf_restart") else 200_000
        if mode_expected is None and stamp < start + grace_us:
            continue
        corrections.append([float(debug[axis][i]) for axis in "xyz"])
    if any(not math.isfinite(value) for vector in corrections for value in vector):
        return {"classification": "FAIL", "reason": "nonfinite correction"}
    peak = max((abs(value) for vector in corrections for value in vector), default=None)
    precursor_peak = max((abs(float(debug[axis][i]))
                          for i, stamp in enumerate(debug["timestamp"])
                          if start - 2_000_000 <= int(stamp) < start
                          and name_at(debug, i) == b"AFCR_DA" for axis in "xyz"), default=None)
    mode_observed = mode_expected is None or mode_expected in modes
    if peak is not None and peak > 1e-4:
        classification = "FAIL"
        reason = "candidate correction nonzero in forbidden phase"
    elif len(corrections) < 100 or not mode_observed:
        classification = "PARTIAL"
        reason = "insufficient zero samples or intended mode not observed"
    else:
        classification = "PASS"
        reason = "zero correction with mode and diagnostic samples"
    gps_samples_after = None
    if event["name"] == "gps_loss":
        try:
            gps = evaluate.dataset(log, "sensor_gps")
            gps_samples_after = sum(int(stamp) >= start + 500_000 for stamp in gps["timestamp"])
        except ValueError:
            gps_samples_after = None
        if gps_samples_after is None:
            classification = "PARTIAL"
            reason = "GPS data topic absent; dropout cannot be verified"
        elif gps_samples_after:
            classification = "PARTIAL"
            reason = "GPS stream continued after fault injection"
    failsafe_observed = None
    if event["name"] == "failsafe":
        failsafe_observed = any(bool(status["failsafe"][i]) for i, stamp in enumerate(status["timestamp"])
                                if start <= int(stamp) < end)
        if not failsafe_observed:
            classification = "PARTIAL"
            reason = "failsafe state not observed"
    ekf_update_gap_s_max = None
    if event["name"] == "ekf_restart":
        estimate = evaluate.dataset(log, "vehicle_local_position")
        stamps = [int(stamp) for stamp in estimate["timestamp"]
                  if start - 1_000_000 <= int(stamp) < end]
        ekf_update_gap_s_max = max(((b - a) / 1e6 for a, b in zip(stamps, stamps[1:])), default=0)
        if ekf_update_gap_s_max < 0.3:
            classification = "PARTIAL"
            reason = "EKF output interruption not observed"
    if event.get("command_rc", 0) != 0:
        classification = "PARTIAL"
        reason = "boundary command failed"
    if event["name"] in PRECURSOR_REQUIRED and (precursor_peak is None or precursor_peak <= 1e-4):
        if classification == "PASS":
            classification = "PARTIAL"
            reason = "candidate was not active before the boundary"
    return {"classification": classification, "reason": reason,
            "correction_samples": len(corrections), "max_abs_correction_m_s2": peak,
            "precursor_max_abs_correction_m_s2": precursor_peak,
            "nav_states_observed": sorted(set(modes)), "expected_nav_state": mode_expected,
            "gps_samples_after_injection": gps_samples_after,
            "failsafe_observed": failsafe_observed,
            "ekf_update_gap_s_max": ekf_update_gap_s_max,
            "command_rc": event.get("command_rc")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args = parser.parse_args()
    trials = json.loads((args.run / "results.json").read_text(encoding="utf-8"))
    findings = []
    for trial in trials:
        item = {"boundary": trial["boundary"], "ulog": trial.get("ulog"),
                "raw_ulog_paths": trial.get("raw_ulog_paths", []), "events": []}
        if "ulog" not in trial:
            item.update(classification="PARTIAL", reason=trial.get("error", "no ULog"))
        else:
            for event in trial["events"]:
                if "start_s" in event:
                    item["events"].append({"name": event["name"], **score_boundary(trial["ulog"], event)})
            classifications = {event["classification"] for event in item["events"]}
            item["classification"] = ("FAIL" if "FAIL" in classifications else
                                      "PARTIAL" if "PARTIAL" in classifications else "PASS")
        findings.append(item)
    (args.run / "analysis.json").write_text(json.dumps(findings, indent=2, sort_keys=True) + "\n",
                                               encoding="utf-8")


if __name__ == "__main__":
    main()
