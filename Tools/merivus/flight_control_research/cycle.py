#!/usr/bin/env python3
"""Run a reproducible candidate-versus-PX4 SITL research cycle on Linux."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import candidate
import evaluate
import shadow


ROOT = Path(__file__).resolve().parents[3]
SPEC = Path(__file__).with_name("candidate.json")
HEADER = ROOT / "src/modules/mc_pos_control/PositionControl/ResearchCandidateSpec.hpp"
ALLOWED_EDITS = {
    "Tools/merivus/flight_control_research/candidate.json",
    "src/modules/mc_pos_control/PositionControl/ResearchCandidateSpec.hpp",
}


def run(command, *, cwd=ROOT):
    subprocess.run(command, cwd=cwd, check=True)


def changed_files():
    changed = subprocess.check_output(["git", "diff", "--name-only", "HEAD"], cwd=ROOT, text=True).splitlines()
    untracked = subprocess.check_output(["git", "ls-files", "--others", "--exclude-standard"],
                                        cwd=ROOT, text=True).splitlines()
    return set(changed + untracked)


def mode_order_for_seed(seed):
    return ("active", "off") if seed % 2 else ("off", "active")


def run_trial(output, dialect, scenario, mode, seed):
    run([sys.executable, str(SPEC.with_name("run_sitl.py")), "--repo", str(ROOT),
         "--output", str(output), "--dialect", str(dialect),
         "--mode", mode, "--scenario", scenario, "--seed", str(seed)])
    ulog = Path((output / "ulog_path.txt").read_text(encoding="utf-8").strip())
    return ulog, evaluate.evaluate(ulog, output / "windows.json")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dialect", type=Path, required=True)
    parser.add_argument("--seeds", type=int, nargs="+", default=[1, 2])
    args = parser.parse_args()
    if os.name != "posix":
        raise SystemExit("PX4 Gazebo Classic cycle requires a Linux host")
    if len(set(args.seeds)) != len(args.seeds) or any(seed < 0 for seed in args.seeds):
        raise ValueError("seeds must be distinct nonnegative integers")
    unexpected = changed_files() - ALLOWED_EDITS
    if unexpected:
        raise RuntimeError("research infrastructure changed: " + ", ".join(sorted(unexpected)))
    spec = json.loads(SPEC.read_text(encoding="utf-8"))
    generated = candidate.generate(spec)
    if HEADER.read_text(encoding="utf-8") != generated:
        raise RuntimeError("generated candidate header differs from candidate.json")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    scripts = ("candidate.py", "evaluate.py", "run_sitl.py", "cycle.py", "research.py", "shadow.py",
               "test_research.py")
    manifest = {
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "candidate_sha256": hashlib.sha256(SPEC.read_bytes()).hexdigest(),
        "scripts_sha256": {name: hashlib.sha256(SPEC.with_name(name).read_bytes()).hexdigest()
                           for name in scripts},
        "seeds": args.seeds,
        "mode_order_by_seed": {str(seed): mode_order_for_seed(seed)
                               for seed in args.seeds},
        "maximum_truth_frame_attempts": 3,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    run([sys.executable, "-m", "unittest", "discover", "-s", str(SPEC.parent), "-p", "test_research.py"])
    run(["make", "px4_sitl_default", "-j4"])
    run(["make", "px4_sitl_default", "sitl_gazebo-classic", "-j4"])
    invalid_trials = []

    def record_invalid(seed, attempt, trial_output, error):
        invalid_trials.append({"seed": seed, "attempt": attempt,
                               "trial": str(trial_output), "reason": str(error)})
        (output / "invalid_trials.json").write_text(
            json.dumps(invalid_trials, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    for attempt in range(1, 4):
        shadow_output = output / f"shadow_gust_seed{args.seeds[0]}_attempt{attempt}"
        try:
            shadow_ulog, shadow_metrics = run_trial(shadow_output, args.dialect.resolve(),
                                                    "gust", "shadow", args.seeds[0])
            break
        except evaluate.InvalidTruthFrame as error:
            record_invalid(args.seeds[0], attempt, shadow_output, error)
    else:
        raise RuntimeError("shadow truth frame invalid after three attempts")

    shadow_result = shadow.verify(shadow_ulog, shadow_output / "windows.json", SPEC)
    if not shadow_metrics["gust_hover"]["hard_gate_passed"]:
        raise RuntimeError("shadow gust trial failed the hover hard gate")
    (output / "shadow.json").write_text(json.dumps({"correction": shadow_result,
                                                      "metrics": shadow_metrics}, indent=2) + "\n",
                                          encoding="utf-8")
    paired = []
    for seed in args.seeds:
        for attempt in range(1, 4):
            baseline, active = {}, {}
            quiescence = {}
            collections = {"off": baseline, "active": active}
            try:
                for scenario in ("normal", "wind", "gust", "payload"):
                    for mode in mode_order_for_seed(seed):
                        collection = collections[mode]
                        trial_output = output / f"seed{seed}_attempt{attempt}_{scenario}_{mode}"
                        try:
                            ulog, metrics = run_trial(trial_output, args.dialect.resolve(),
                                                      scenario, mode, seed)
                        except evaluate.InvalidTruthFrame as error:
                            record_invalid(seed, attempt, trial_output, error)
                            raise
                        collection.update(metrics)
                        if mode == "active" and scenario != "gust":
                            quiescence[scenario + "_hover"] = shadow.verify_quiescent(
                                ulog, trial_output / "windows.json")
            except evaluate.InvalidTruthFrame:
                continue
            break
        else:
            raise RuntimeError(f"seed {seed} truth frame invalid after three attempts")
        decision = evaluate.compare(baseline, active, quiescence)
        paired.append({"seed": seed, "attempt": attempt, "baseline": baseline, "active": active,
                       "quiescence": quiescence, "decision": decision})
        (output / "results.json").write_text(json.dumps(paired, indent=2, sort_keys=True) + "\n",
                                                encoding="utf-8")
    retained = all(item["decision"]["retain"] for item in paired)
    (output / "decision.json").write_text(json.dumps({"retain_for_further_sitl": retained,
                                                      "seed_count": len(paired)}, indent=2) + "\n",
                                              encoding="utf-8")
    print("retain for further SITL" if retained else "reject candidate")


if __name__ == "__main__":
    main()
