#!/usr/bin/env python3
"""Verify that a SITL shadow trial logged finite, bounded candidate corrections."""

import argparse
import json
import math
from pathlib import Path


def verify_samples(data, window, spec):
    limit = spec["nodes"][-1]["limit_m_s2"]
    maximum = spec["maximum_correction_m_s2"]
    start = int(window["start_s"] * 1_000_000)
    end = int(window["end_s"] * 1_000_000)
    peaks = [0.0, 0.0, 0.0]
    squares = [0.0, 0.0, 0.0]
    samples = saturation = nonzero = 0
    for index, timestamp in enumerate(data["timestamp"]):
        if not start <= int(timestamp) < end:
            continue
        name = bytes(int(data[f"name[{byte}]"][index]) for byte in range(10)).split(b"\0", 1)[0]
        if name != b"AFCR_DA":
            continue
        correction = [float(data[axis][index]) for axis in "xyz"]
        if not all(math.isfinite(value) for value in correction):
            raise ValueError("shadow correction contains a nonfinite value")
        if any(abs(value) > min(bound, maximum) + 1e-4
               for value, bound in zip(correction, limit)):
            raise ValueError("shadow correction exceeded candidate limit")
        samples += 1
        nonzero += any(abs(value) > 0.005 for value in correction)
        saturation += any(abs(value) >= bound - 0.001
                          for value, bound in zip(correction, limit))
        for axis, value in enumerate(correction):
            peaks[axis] = max(peaks[axis], abs(value))
            squares[axis] += value * value
    if samples < 500 or nonzero < 100:
        raise ValueError(f"shadow correction insufficient: samples={samples}, nonzero={nonzero}")
    return {"samples": samples, "nonzero_samples": nonzero,
            "max_abs_m_s2": peaks,
            "rms_m_s2": [math.sqrt(total / samples) for total in squares],
            "saturation_fraction": saturation / samples}


def verify(ulog_path, windows_path, spec_path):
    from pyulog import ULog

    log = ULog(str(ulog_path))
    matches = [item for item in log.data_list if item.name == "debug_vect" and item.multi_id == 0]
    if len(matches) != 1:
        raise ValueError("shadow debug_vect missing or ambiguous")
    windows = json.loads(windows_path.read_text(encoding="utf-8"))
    if len(windows) != 1 or windows[0]["name"] != "gust_hover":
        raise ValueError("one gust_hover window required for shadow verification")
    spec = json.loads(spec_path.read_text(encoding="utf-8"))
    return verify_samples(matches[0].data, windows[0], spec)


def verify_quiescent_samples(data, window):
    start = int(window["start_s"] * 1_000_000)
    end = int(window["end_s"] * 1_000_000)
    recorded = scored = 0
    peak = 0.0
    for index, timestamp in enumerate(data["timestamp"]):
        if int(timestamp) >= end:
            continue
        name = bytes(int(data[f"name[{byte}]"][index]) for byte in range(10)).split(b"\0", 1)[0]
        if name != b"AFCR_DA":
            continue
        correction = [float(data[axis][index]) for axis in "xyz"]
        if not all(math.isfinite(value) for value in correction):
            raise ValueError("candidate correction contains a nonfinite value")
        recorded += 1
        scored += int(timestamp) >= start
        peak = max(peak, *(abs(value) for value in correction))
    if scored < 500:
        raise ValueError(f"candidate correction logging insufficient: scored={scored}")
    if peak > 0.0001:
        raise ValueError(f"candidate correction active outside gust: peak={peak}")
    return {"recorded_samples": recorded, "scored_samples": scored,
            "max_abs_m_s2": peak}


def verify_quiescent(ulog_path, windows_path):
    from pyulog import ULog

    windows = json.loads(windows_path.read_text(encoding="utf-8"))
    if len(windows) != 1 or windows[0]["name"] not in (
            "normal_hover", "wind_hover", "payload_hover"):
        raise ValueError("one non-gust hover window required")
    log = ULog(str(ulog_path))
    matches = [item for item in log.data_list if item.name == "debug_vect" and item.multi_id == 0]
    if len(matches) != 1:
        raise ValueError("candidate correction debug_vect missing or ambiguous")
    return verify_quiescent_samples(matches[0].data, windows[0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ulog", type=Path, required=True)
    parser.add_argument("--windows", type=Path, required=True)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = verify(args.ulog, args.windows, args.spec)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
