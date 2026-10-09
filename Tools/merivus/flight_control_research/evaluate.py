#!/usr/bin/env python3
"""Score a fixed hover window from SITL ULog truth and allocator telemetry."""

import argparse
import bisect
import json
import math
from pathlib import Path
import statistics


class InvalidTruthFrame(ValueError):
    """The logged truth is not the local, centimetre-resolution SITL frame."""


def dataset(log, name):
    matches = [item for item in log.data_list if item.name == name and item.multi_id == 0]
    if len(matches) != 1:
        raise ValueError(f"required ULog topic missing or ambiguous: {name}")
    return matches[0].data


def sample_at(data, timestamp, maximum_age_us=100_000):
    times = data["timestamp"]
    index = bisect.bisect_right(times, timestamp) - 1
    if index < 0 or timestamp - int(times[index]) > maximum_age_us:
        return None
    return index


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def score_arrays(truth, reference, allocator, motors, status, start_us, end_us,
                 event_start_us=None, event_end_us=None):
    if end_us <= start_us:
        raise ValueError("window end must follow start")
    xy_errors, z_errors, efforts = [], [], []
    saturation = 0
    samples = 0
    invalid_mode = 0
    truth_points = []
    for i, timestamp in enumerate(truth["timestamp"]):
        timestamp = int(timestamp)
        if timestamp < start_us or timestamp >= end_us:
            continue
        truth_position = [float(truth[axis][i]) for axis in ("x", "y", "z")]
        if not all(math.isfinite(value) and abs(value) < 100.0 for value in truth_position):
            raise InvalidTruthFrame("groundtruth coordinate frame invalid for local hover scoring")
        j = sample_at(reference, timestamp, 150_000)
        k = sample_at(allocator, timestamp, 300_000)
        m = sample_at(motors, timestamp, 150_000)
        s = sample_at(status, timestamp, 1_000_000)
        if j is None or k is None or m is None or s is None:
            continue
        error = [value - float(reference[axis][j])
                 for value, axis in zip(truth_position, ("x", "y", "z"))]
        controls = [float(motors[f"control[{axis}]"][m]) for axis in range(4)]
        if not all(math.isfinite(value) for value in error + controls):
            raise ValueError("nonfinite truth, reference, or motor output")
        xy_errors.append(math.hypot(error[0], error[1]))
        z_errors.append(abs(error[2]))
        truth_points.append((timestamp, *truth_position))
        efforts.append(sum(value * value for value in controls))
        saturation += not (bool(allocator["thrust_setpoint_achieved"][k])
                           and bool(allocator["torque_setpoint_achieved"][k]))
        invalid_mode += not (int(status["arming_state"][s]) == 2
                             and int(status["nav_state"][s]) in (2, 4))
        samples += 1
    expected = (end_us - start_us) / 20_000
    if samples < 20 or samples < expected * 0.6:
        raise ValueError(f"insufficient aligned truth samples: {samples}")
    anchor_points = [point for point in truth_points if point[0] < start_us + 2_000_000]
    if len(anchor_points) < 20:
        raise ValueError("insufficient truth samples to anchor the hover position")
    anchor = [statistics.median(point[axis] for point in anchor_points) for axis in (1, 2, 3)]
    station_xy = [math.hypot(point[1] - anchor[0], point[2] - anchor[1]) for point in truth_points]
    station_z = [abs(point[3] - anchor[2]) for point in truth_points]
    metrics = {
        "samples": samples,
        "xy_rmse_m": rms(xy_errors),
        "z_rmse_m": rms(z_errors),
        "xy_peak_m": max(xy_errors),
        "z_peak_m": max(z_errors),
        "station_xy_rmse_m": rms(station_xy),
        "station_z_rmse_m": rms(station_z),
        "station_xy_peak_m": max(station_xy),
        "station_z_peak_m": max(station_z),
        "motor_effort_mean": sum(efforts) / samples,
        "allocation_failure_fraction": saturation / samples,
        "invalid_mode_fraction": invalid_mode / samples,
    }
    if event_start_us is not None or event_end_us is not None:
        if (event_start_us is None or event_end_us is None or
                not start_us + 2_000_000 <= event_start_us < event_end_us < end_us):
            raise ValueError("event must follow the truth anchor and end within the scored window")
        event_indices = [index for index, point in enumerate(truth_points)
                         if point[0] >= event_start_us]
        recovery_indices = [index for index in event_indices
                            if truth_points[index][0] >= event_end_us]
        if len(event_indices) < 20 or len(recovery_indices) < 20:
            raise ValueError("insufficient truth samples for event scoring")
        metrics.update(event_xy_rmse_m=rms([station_xy[index] for index in event_indices]),
                       event_z_rmse_m=rms([station_z[index] for index in event_indices]),
                       event_xy_peak_m=max(station_xy[index] for index in event_indices),
                       event_z_peak_m=max(station_z[index] for index in event_indices),
                       recovery_xy_rmse_m=rms([station_xy[index] for index in recovery_indices]),
                       recovery_z_rmse_m=rms([station_z[index] for index in recovery_indices]))
    metrics["hard_gate_passed"] = (metrics["xy_peak_m"] < 2.0
                                    and metrics["z_peak_m"] < 1.0
                                    and metrics["station_xy_peak_m"] < 2.0
                                    and metrics["station_z_peak_m"] < 1.0
                                    and metrics["allocation_failure_fraction"] < 0.05
                                    and metrics["invalid_mode_fraction"] == 0)
    return metrics


def evaluate(ulog_path, windows_path):
    from pyulog import ULog
    log = ULog(str(ulog_path))
    truth = dataset(log, "vehicle_local_position_groundtruth")
    reference = dataset(log, "vehicle_local_position_setpoint")
    allocator = dataset(log, "control_allocator_status")
    motors = dataset(log, "actuator_motors")
    status = dataset(log, "vehicle_status")
    windows = json.loads(windows_path.read_text(encoding="utf-8"))
    if not isinstance(windows, list) or not windows:
        raise ValueError("windows must be a nonempty list")
    result = {}
    for window in windows:
        name = window["name"]
        if name in result:
            raise ValueError(f"duplicate window: {name}")
        result[name] = score_arrays(truth, reference, allocator, motors, status,
                                    int(window["start_s"] * 1_000_000),
                                    int(window["end_s"] * 1_000_000),
                                    int(window["event_start_s"] * 1_000_000)
                                    if "event_start_s" in window else None,
                                    int(window["event_end_s"] * 1_000_000)
                                    if "event_end_s" in window else None)
    return result


def compare(baseline, candidate, quiescence):
    if baseline.keys() != candidate.keys():
        raise ValueError("baseline and candidate windows differ")
    if not all(item["hard_gate_passed"] for item in baseline.values()):
        return {"retain": False, "reason": "baseline hard gate failed"}
    if not all(item["hard_gate_passed"] for item in candidate.values()):
        return {"retain": False, "reason": "candidate hard gate failed"}
    quiet_scenarios = {"normal_hover", "wind_hover", "payload_hover"}
    if not quiet_scenarios.issubset(baseline) or set(quiescence) != quiet_scenarios:
        raise ValueError("normal, wind and payload quiescence required")
    if any(item["max_abs_m_s2"] > 0.0001 or item["scored_samples"] < 500
           for item in quiescence.values()):
        return {"retain": False, "reason": "candidate active outside gust"}
    gust = "gust_hover"
    if gust not in baseline:
        raise ValueError("gust_hover window required")
    if candidate[gust]["event_z_rmse_m"] > baseline[gust]["event_z_rmse_m"] + 0.050:
        return {"retain": False, "reason": "gust vertical regression"}
    if candidate[gust]["motor_effort_mean"] > baseline[gust]["motor_effort_mean"] * 1.05:
        return {"retain": False, "reason": "gust motor effort regression"}
    if candidate[gust]["event_xy_rmse_m"] > baseline[gust]["event_xy_rmse_m"] - 0.080:
        return {"retain": False, "reason": "insufficient gust XY improvement"}
    if candidate[gust]["recovery_xy_rmse_m"] > baseline[gust]["recovery_xy_rmse_m"] - 0.050:
        return {"retain": False, "reason": "insufficient gust recovery improvement"}
    return {"retain": True, "reason": "gust displacement and recovery improved"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ulog", type=Path, required=True)
    parser.add_argument("--windows", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(args.ulog, args.windows)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
