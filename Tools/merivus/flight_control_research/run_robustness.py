#!/usr/bin/env python3
"""Run frozen, paired AFCR robustness cases in independent PX4/Gazebo sessions."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import evaluate


ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent


def save(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_case(output, dialect, case, mode):
    scenario = "wind" if case["disturbance"] == "wind" else "gust"
    subprocess.run([sys.executable, str(HERE / "run_sitl.py"), "--repo", str(ROOT),
                    "--output", str(output), "--dialect", str(dialect),
                    "--mode", mode, "--scenario", scenario,
                    "--seed", str(case["seed"]), "--case", str(output.parent / "case.json")],
                   cwd=ROOT, check=True)
    ulog = Path((output / "ulog_path.txt").read_text(encoding="utf-8").strip())
    return {"ulog": str(ulog), "metrics": evaluate.evaluate(ulog, output / "windows.json")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dialect", type=Path, required=True)
    parser.add_argument("--ids", nargs="*")
    args = parser.parse_args()
    if os.name != "posix":
        raise SystemExit("requires Linux PX4/Gazebo SITL")
    matrix = json.loads((HERE / "robustness_matrix.json").read_text(encoding="utf-8"))
    spec_hash = hashlib.sha256((HERE / "candidate.json").read_bytes()).hexdigest()
    if spec_hash != matrix["candidate_sha256"]:
        raise RuntimeError("candidate 006 changed after matrix registration")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    selected = [case for case in matrix["cases"] if not args.ids or case["id"] in args.ids]
    if args.ids and set(args.ids) != {case["id"] for case in selected}:
        raise ValueError("unknown or duplicate case ID")
    save(output / "manifest.json", {
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "candidate_sha256": spec_hash,
        "matrix_sha256": hashlib.sha256((HERE / "robustness_matrix.json").read_bytes()).hexdigest(),
        "script_sha256": {name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
                          for name in ("run_sitl.py", "run_robustness.py", "evaluate.py", "wind_profile_plugin.cpp")},
        "selected_ids": [case["id"] for case in selected],
    })
    results = []
    for case in selected:
        case_output = output / case["id"]
        case_output.mkdir()
        save(case_output / "case.json", case)
        for attempt in range(1, 4):
            pair = {"id": case["id"], "attempt": attempt, "trials": {}}
            order = ("active", "off") if case["seed"] % 2 else ("off", "active")
            invalid_truth = False
            for mode in order:
                trial_output = case_output / f"attempt{attempt}_{mode}"
                try:
                    pair["trials"][mode] = run_case(trial_output, args.dialect.resolve(), case, mode)
                except evaluate.InvalidTruthFrame as error:
                    pair["trials"][mode] = {"status": "INVALID_TRUTH", "error": str(error)}
                    invalid_truth = True
                    break
                except Exception as error:
                    pair["trials"][mode] = {"status": "ERROR", "error": str(error)}
                    results.append(pair)
                    save(output / "results.json", results)
                    raise
            results.append(pair)
            save(output / "results.json", results)
            if not invalid_truth:
                break
        else:
            raise RuntimeError(f"{case['id']}: invalid truth after three attempts")
    print(output / "results.json")


if __name__ == "__main__":
    main()
