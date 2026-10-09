#!/usr/bin/env python3
"""Exercise SITL-only AFCR safety gates during scripted flight-state changes."""

import argparse
import json
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace

from run_sitl import Trial, hover_reached


BOUNDARIES = (
    "takeoff", "landing", "manual_stick", "position_to_altitude",
    "position_to_stabilized", "stabilized_to_position", "rtl", "failsafe",
    "gps_loss", "ekf_restart",
)


def save(path, data):
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def cli_while_pumping(trial, module, *args):
    command = [str(trial.build / "bin" / ("px4-" + module)), *map(str, args)]
    process = subprocess.Popen(command, cwd=trial.rootfs, env=trial.env,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    deadline = time.monotonic() + 30
    try:
        while process.poll() is None:
            trial.pump()
            if time.monotonic() > deadline:
                raise TimeoutError(f"PX4 command timed out: {module} {' '.join(args)}")
            time.sleep(.01)
        stdout, _ = process.communicate(timeout=2)
        return SimpleNamespace(returncode=process.returncode, stdout=stdout)
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=2)


def run_boundary(repo, output, dialect, boundary, seed):
    case_path = output.parent / "case.json"
    save(case_path, {"disturbance": "gust", "magnitude_m_s": 8,
                     "direction": "1 0 0", "duration_s": 8})
    trial = Trial(SimpleNamespace(repo=repo, output=output, dialect=dialect, mode="active",
                                  scenario="gust", seed=seed, case=case_path))
    events = []
    try:
        trial.spawn(["gzserver", "--verbose", "--seed", str(seed), str(trial.world)], "gazebo")
        time.sleep(3)
        spawned = subprocess.run(["gz", "model", "--spawn-file=" + str(trial.model),
                                  "--model-name=iris", "-x", "0", "-y", "0", "-z", ".5"],
                                 env=trial.env, capture_output=True, timeout=30)
        (trial.output / "spawn.log").write_bytes(spawned.stdout + spawned.stderr)
        if spawned.returncode:
            raise RuntimeError("Gazebo model spawn failed")
        trial.spawn([str(trial.build / "bin/px4"), "-d", "-w", str(trial.rootfs),
                     str(trial.build / "etc")], "px4")
        deadline = time.monotonic() + 60
        while "LOCAL_POSITION_NED" not in trial.latest:
            if time.monotonic() > deadline:
                raise TimeoutError("PX4 local position unavailable")
            trial.pump()
        trial.wait_for(12)
        takeoff_z = trial.latest["LOCAL_POSITION_NED"].z
        trial.writer.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
        trial.wait_for(2)
        if trial.cli("commander", "takeoff").returncode:
            raise RuntimeError("PX4 takeoff command failed")
        events.append({"name": "takeoff", "start_s": trial.sim_time, "end_s": trial.sim_time + 8})
        trial.wait_for(20)
        if not hover_reached(trial.latest.get("LOCAL_POSITION_NED"), takeoff_z):
            raise RuntimeError("stable hover not reached")
        if boundary == "stabilized_to_position":
            trial.wait_for(max(0, 44 - trial.sim_time))
            result = trial.cli("commander", "mode", "stabilized")
            events.append({"name": "enter_stabilized", "time_s": trial.sim_time,
                           "command_rc": result.returncode, "stdout": result.stdout.strip()})
            trial.wait_for(3)
            result = trial.cli("commander", "mode", "posctl")
            events.append({"name": boundary, "start_s": trial.sim_time,
                           "end_s": trial.sim_time + 2, "command_rc": result.returncode,
                           "stdout": result.stdout.strip()})
            trial.wait_for(7)
        elif boundary not in ("takeoff", "landing"):
            trial.wait_for(max(0, 48 - trial.sim_time))
            start = trial.sim_time
            if boundary == "manual_stick":
                trial.manual_x = 400
                result = SimpleNamespace(returncode=0, stdout="manual x=400")
            elif boundary == "position_to_altitude":
                result = trial.cli("commander", "mode", "altctl")
            elif boundary == "position_to_stabilized":
                result = trial.cli("commander", "mode", "stabilized")
            elif boundary == "rtl":
                result = trial.cli("commander", "mode", "auto:rtl")
            elif boundary == "failsafe":
                trial.manual_enabled = False
                result = SimpleNamespace(returncode=0, stdout="manual-control MAVLink stopped")
            elif boundary == "gps_loss":
                trial.writer.command_long_send(1, 1, 420, 0, 4, 1, 0, 0, 0, 0, 0)
                result = SimpleNamespace(returncode=0, stdout="MAV_CMD_INJECT_FAILURE GPS OFF")
            elif boundary == "ekf_restart":
                result = cli_while_pumping(trial, "ekf2", "stop")
                trial.wait_for(1)
                restarted = cli_while_pumping(trial, "ekf2", "start")
                result = SimpleNamespace(returncode=max(result.returncode, restarted.returncode),
                                         stdout=result.stdout + restarted.stdout)
            observation_s = 8 if boundary in ("failsafe", "gps_loss") else 5
            events.append({"name": boundary, "start_s": start,
                           "end_s": start + observation_s, "command_rc": result.returncode,
                           "stdout": result.stdout.strip()})
            trial.wait_for(observation_s)
        if boundary == "landing":
            trial.wait_for(max(0, 48 - trial.sim_time))
            start = trial.sim_time
            result = trial.cli("commander", "land")
            events.append({"name": "landing", "start_s": start,
                           "end_s": start + 6, "command_rc": result.returncode,
                           "stdout": result.stdout.strip()})
            trial.wait_for(12)
        else:
            trial.cli("commander", "land")
            trial.wait_for(12)
        trial.cli("logger", "stop")
        logs = sorted((trial.rootfs / "log").rglob("*.ulg"))
        if len(logs) != 1:
            raise RuntimeError(f"expected one ULog, got {len(logs)}")
        (trial.output / "ulog_path.txt").write_text(str(logs[0]) + "\n", encoding="utf-8")
        save(trial.output / "events.json", events)
        return {"boundary": boundary, "ulog": str(logs[0]), "events": events}
    finally:
        trial.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dialect", type=Path, required=True)
    parser.add_argument("--ids", nargs="*")
    args = parser.parse_args()
    selected = args.ids or BOUNDARIES
    if len(set(selected)) != len(selected) or not set(selected).issubset(BOUNDARIES):
        raise ValueError("unknown or duplicate safety boundary")
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    for index, boundary in enumerate(selected):
        path = args.output / boundary
        try:
            result = run_boundary(args.repo.resolve(), path, args.dialect.resolve(), boundary, 71 + index)
        except Exception as error:
            result = {"boundary": boundary, "status": "ERROR", "error": str(error),
                      "trial_output": str(path),
                      "raw_ulog_paths": [str(log) for log in sorted((path / "rootfs/log").rglob("*.ulg"))]}
        results.append(result)
        save(args.output / "results.json", results)
    print(args.output / "results.json")


if __name__ == "__main__":
    main()
