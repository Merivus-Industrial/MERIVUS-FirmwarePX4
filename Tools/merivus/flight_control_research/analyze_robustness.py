#!/usr/bin/env python3
"""Summarize per-case worst-tail and activation data from frozen AFCR SITL ULogs."""

import argparse
import bisect
import json
import math
from pathlib import Path
import statistics

from pyulog import ULog

import evaluate


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def aligned(data, timestamp, max_age=150_000):
    index = bisect.bisect_right(data["timestamp"], timestamp) - 1
    if index < 0 or timestamp - int(data["timestamp"][index]) > max_age:
        return None
    return index


def debug_name(data, index):
    return bytes(int(data[f"name[{byte}]"][index]) for byte in range(10)).split(b"\0", 1)[0]


def score_trial(ulog, window):
    log = ULog(str(ulog))
    truth = evaluate.dataset(log, "vehicle_local_position_groundtruth")
    reference = evaluate.dataset(log, "vehicle_local_position_setpoint")
    estimate = evaluate.dataset(log, "vehicle_local_position")
    attitude = evaluate.dataset(log, "vehicle_attitude")
    motors = evaluate.dataset(log, "actuator_motors")
    debug = evaluate.dataset(log, "debug_vect")
    start = int(window["start_s"] * 1e6)
    end = int(window["end_s"] * 1e6)
    positions = []
    errors = []
    speeds = []
    velocity_errors = []
    tilt = []
    motor_peak = []
    for i, stamp in enumerate(truth["timestamp"]):
        stamp = int(stamp)
        if not start <= stamp < end:
            continue
        position = [float(truth[axis][i]) for axis in "xyz"]
        if not all(math.isfinite(value) and abs(value) < 100 for value in position):
            raise evaluate.InvalidTruthFrame("groundtruth coordinate frame invalid")
        j = aligned(reference, stamp)
        k = aligned(estimate, stamp)
        a = aligned(attitude, stamp)
        m = aligned(motors, stamp)
        if None in (j, k, a, m):
            continue
        xy_error = math.hypot(position[0] - float(reference["x"][j]),
                              position[1] - float(reference["y"][j]))
        vx, vy = float(estimate["vx"][k]), float(estimate["vy"][k])
        evx = float(reference["vx"][j]) - vx
        evy = float(reference["vy"][j]) - vy
        q0, q1, q2 = (float(attitude[f"q[{axis}]"][a]) for axis in range(3))
        q3 = float(attitude["q[3]"][a])
        body_z = 1 - 2 * (q1*q1 + q2*q2)
        values = (xy_error, vx, vy, evx, evy, q0, q1, q2, q3, body_z)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("nonfinite trajectory or estimate")
        positions.append((stamp, position))
        errors.append(xy_error)
        speeds.append(math.hypot(vx, vy))
        velocity_errors.append(math.hypot(evx, evy))
        tilt.append(math.degrees(math.acos(max(-1.0, min(1.0, body_z)))))
        motor_peak.append(max(float(motors[f"control[{axis}]"][m]) for axis in range(4)))
    if len(positions) < (end - start) / 20_000 * 0.6:
        raise ValueError("insufficient aligned samples for extended metrics")
    anchor = [statistics.median(point[1][axis] for point in positions
                                if point[0] < start + 2_000_000) for axis in (0, 1)]
    excursion = [math.hypot(point[1][0] - anchor[0], point[1][1] - anchor[1])
                 for point in positions]
    event_end = int(window["event_end_s"] * 1e6) if "event_end_s" in window else None
    recovery = [(stamp, error) for (stamp, _), error in zip(positions, errors)
                if event_end is not None and stamp >= event_end]
    settling = None
    if recovery:
        for stamp, error in recovery:
            if error <= 0.2 and all(value <= 0.2 for time, value in recovery
                                    if stamp <= time <= stamp + 2_000_000):
                if recovery[-1][0] >= stamp + 2_000_000:
                    settling = (stamp - event_end) / 1e6
                    break
    correction = []
    full_correction_peak = 0.0
    for i, stamp in enumerate(debug["timestamp"]):
        stamp = int(stamp)
        if stamp >= end or debug_name(debug, i) != b"AFCR_DA":
            continue
        vector = [float(debug[axis][i]) for axis in "xyz"]
        if not all(math.isfinite(value) for value in vector):
            raise ValueError("nonfinite candidate correction")
        full_correction_peak = max(full_correction_peak, *(abs(value) for value in vector))
        if stamp >= start:
            correction.append((stamp, vector))
    if len(correction) < 0.6 * (end - start) / 20_000:
        raise ValueError("insufficient AFCR_DA samples")
    norms = [math.hypot(value[0], value[1]) for _, value in correction]
    slopes = [math.hypot(*(current[1][axis] - previous[1][axis] for axis in (0, 1))) /
              ((current[0] - previous[0]) / 1e6)
              for previous, current in zip(correction, correction[1:]) if current[0] > previous[0]]
    return {
        "aligned_samples": len(positions), "correction_samples": len(correction),
        "xy_error_p95_m": percentile(errors, 0.95),
        "xy_error_max_m": max(errors),
        "xy_excursion_max_m": max(excursion),
        "xy_velocity_p95_m_s": percentile(speeds, 0.95),
        "velocity_error_p95_m_s": percentile(velocity_errors, 0.95),
        "velocity_error_max_m_s": max(velocity_errors),
        "velocity_gate_partial_or_full_fraction_inferred": sum(value > 0.08 for value in velocity_errors) / len(velocity_errors),
        "velocity_gate_full_fraction_inferred": sum(value >= 0.25 for value in velocity_errors) / len(velocity_errors),
        "gate_state_source": "inferred from aligned ULog velocity error; hover latch not logged",
        "attitude_tilt_max_deg": max(tilt),
        "actuator_output_max": max(motor_peak),
        "candidate_activation_fraction": sum(value > 1e-4 for value in norms) / len(norms),
        "candidate_max_correction_m_s2": max(norms),
        "candidate_max_abs_before_window_end_m_s2": full_correction_peak,
        "candidate_axis_max_m_s2": [max(abs(vector[axis]) for _, vector in correction)
                                      for axis in range(3)],
        "candidate_limit_incidence": sum(abs(vector[0]) >= 0.349 or abs(vector[1]) >= 0.349
                                         for _, vector in correction) / len(correction),
        "candidate_slope_p95_m_s3": percentile(slopes, 0.95),
        "post_event_overshoot_m": max((value for (stamp, _), value in zip(positions, excursion)
                                        if event_end is not None and stamp >= event_end), default=None),
        "settling_time_s": settling,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args = parser.parse_args()
    results = json.loads((args.run / "results.json").read_text(encoding="utf-8"))
    extended = []
    for pair in results:
        item = {"id": pair["id"], "attempt": pair["attempt"], "trials": {}}
        for mode, trial in pair["trials"].items():
            if "ulog" not in trial:
                item["trials"][mode] = trial
                continue
            trial_path = args.run / pair["id"] / f"attempt{pair['attempt']}_{mode}"
            window = json.loads((trial_path / "windows.json").read_text(encoding="utf-8"))[0]
            item["trials"][mode] = score_trial(Path(trial["ulog"]), window)
        extended.append(item)
    (args.run / "extended.json").write_text(json.dumps(extended, indent=2, sort_keys=True) + "\n",
                                              encoding="utf-8")


if __name__ == "__main__":
    main()
