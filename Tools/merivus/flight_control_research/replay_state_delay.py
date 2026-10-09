#!/usr/bin/env python3
"""Offline shadow sensitivity of frozen Candidate 006 to delayed local estimates."""

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

from pyulog import ULog

import evaluate


DELAYS_MS = (0, 20, 50, 100)


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)] if ordered else None


def replay(ulog, window, delay_ms):
    log = ULog(str(ulog))
    estimate = evaluate.dataset(log, "vehicle_local_position")
    reference = evaluate.dataset(log, "vehicle_local_position_setpoint")
    start = int(window["start_s"] * 1e6)
    end = int(window["end_s"] * 1e6)
    fast = [0.0, 0.0]
    slow = [0.0, 0.0]
    last_baseline = None
    last_time = None
    samples = []
    for stamp in estimate["timestamp"]:
        stamp = int(stamp)
        if not start <= stamp < end:
            continue
        i = bisect.bisect_right(reference["timestamp"], stamp) - 1
        if i < 0 or stamp - int(reference["timestamp"][i]) > 100_000:
            continue
        target = stamp - delay_ms * 1000
        k = bisect.bisect_right(estimate["timestamp"], target) - 1
        if k < 0 or target - int(estimate["timestamp"][k]) > 100_000:
            continue
        dt = (stamp - last_time) / 1e6 if last_time is not None else 0.01
        last_time = stamp
        if not 0.002 <= dt <= 0.04:
            fast = [0.0, 0.0]
            slow = [0.0, 0.0]
            last_baseline = None
            continue
        baseline = [float(reference[f"acceleration[{axis}]"][i]) for axis in (0, 1)]
        measured = [float(estimate[axis][k]) for axis in ("ax", "ay")]
        error = [float(reference[axis][i]) - float(estimate[measured_axis][k])
                 for axis, measured_axis in (("vx", "vx"), ("vy", "vy"))]
        if not all(math.isfinite(value) for value in baseline + measured + error):
            continue
        residual = ([measured[axis] - last_baseline[axis] for axis in (0, 1)]
                    if last_baseline is not None else [0.0, 0.0])
        for axis in (0, 1):
            fast[axis] += (residual[axis] - fast[axis]) * dt / (0.12 + dt)
            slow[axis] += (residual[axis] - slow[axis]) * dt / (1.5 + dt)
        q = math.hypot(*error)
        gate = max(0.0, min(1.0, (q - 0.08) / 0.17))
        correction = [max(-0.35, min(0.35,
                          gate * (0.20 * error[axis] - 0.15 * fast[axis] + 0.15 * slow[axis])))
                      for axis in (0, 1)]
        last_baseline = baseline
        samples.append((stamp, correction[0], correction[1], gate, q))
    if len(samples) < 500:
        raise ValueError(f"insufficient replay samples: {len(samples)}")
    slopes = [math.hypot(current[1] - previous[1], current[2] - previous[2]) /
              ((current[0] - previous[0]) / 1e6)
              for previous, current in zip(samples, samples[1:]) if current[0] > previous[0]]
    return {"samples": len(samples), "delay_ms": delay_ms,
            "correction_max_m_s2": max(math.hypot(item[1], item[2]) for item in samples),
            "correction_p95_m_s2": percentile([math.hypot(item[1], item[2]) for item in samples], 0.95),
            "activation_fraction": sum(math.hypot(item[1], item[2]) > 1e-4 for item in samples) / len(samples),
            "velocity_gate_open_fraction": sum(item[3] > 0 for item in samples) / len(samples),
            "velocity_error_p95_m_s": percentile([item[4] for item in samples], 0.95),
            "correction_slope_abs_p95_m_s3": percentile(slopes, 0.95)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--case-id", action="append", default=[])
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    matrix = json.loads((here / "robustness_matrix.json").read_text(encoding="utf-8"))
    if hashlib.sha256((here / "candidate.json").read_bytes()).hexdigest() != matrix["candidate_sha256"]:
        raise RuntimeError("candidate 006 changed; delay replay invalid")
    results = json.loads((args.run / "results.json").read_text(encoding="utf-8"))
    by_id = {pair["id"]: pair for pair in results}
    output = {}
    for case_id in (args.case_id or ["nominal_iris_8", "gps_vel_noise_2x"]):
        pair = by_id[case_id]
        ulog = Path(pair["trials"]["off"]["ulog"])
        trial_path = args.run / case_id / f"attempt{pair['attempt']}_off"
        window = json.loads((trial_path / "windows.json").read_text(encoding="utf-8"))[0]
        output[case_id] = {str(delay): replay(ulog, window, delay) for delay in DELAYS_MS}
    (args.run / "state_delay_replay.json").write_text(json.dumps(output, indent=2, sort_keys=True) + "\n",
                                                       encoding="utf-8")


if __name__ == "__main__":
    main()
