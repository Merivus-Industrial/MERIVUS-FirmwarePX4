#!/usr/bin/env python3
"""Apply the preregistered per-case AFCR robustness limits without averaging cases."""

import argparse
import json
from pathlib import Path


def difference(active, baseline, key):
    a, b = active.get(key), baseline.get(key)
    return None if a is None or b is None else a - b


def classify(pair, extended, case):
    item = {"id": pair["id"], "classification": "PARTIAL", "reasons": [], "observations": [],
            "raw_ulog": {mode: trial.get("ulog") for mode, trial in pair["trials"].items()}}
    if set(pair["trials"]) != {"active", "off"} or any(
            "metrics" not in trial for trial in pair["trials"].values()):
        item["reasons"].append("配对试验不完整")
        return item
    active = pair["trials"]["active"]["metrics"]["gust_hover" if case["disturbance"] != "wind" else "wind_hover"]
    baseline = pair["trials"]["off"]["metrics"]["gust_hover" if case["disturbance"] != "wind" else "wind_hover"]
    ea, eb = extended["trials"]["active"], extended["trials"]["off"]
    if "xy_error_p95_m" not in ea or "xy_error_p95_m" not in eb:
        item["reasons"].append("扩展 ULog 指标不完整")
        return item
    item["metrics"] = {
        "event_xy_rmse_delta_m": difference(active, baseline, "event_xy_rmse_m"),
        "xy_error_p95_delta_m": difference(ea, eb, "xy_error_p95_m"),
        "xy_excursion_max_delta_m": difference(ea, eb, "xy_excursion_max_m"),
        "post_event_overshoot_delta_m": difference(ea, eb, "post_event_overshoot_m"),
        "settling_time_delta_s": difference(ea, eb, "settling_time_s"),
        "candidate_activation_fraction": ea["candidate_activation_fraction"],
        "candidate_max_correction_m_s2": ea["candidate_max_correction_m_s2"],
        "candidate_max_abs_before_window_end_m_s2": ea["candidate_max_abs_before_window_end_m_s2"],
        "candidate_axis_max_m_s2": ea["candidate_axis_max_m_s2"],
        "candidate_limit_incidence": ea["candidate_limit_incidence"],
        "allocator_saturation_active": active["allocation_failure_fraction"],
        "allocator_saturation_off": baseline["allocation_failure_fraction"],
        "candidate_slope_p95_m_s3": ea["candidate_slope_p95_m_s3"],
    }
    failed = []
    if not active["hard_gate_passed"]:
        failed.append("候选安全硬门失败")
    if not baseline["hard_gate_passed"]:
        item["observations"].append("原生 PX4 同样超出预注册安全包线")
        if failed:
            item.update(classification="FAIL", reasons=failed)
        else:
            item["reasons"].append("原生 PX4 超出预注册安全包线")
        return item
    for key, maximum, description in (
        ("xy_error_p95_delta_m", 0.1, "XY 误差 95 分位退化"),
        ("xy_excursion_max_delta_m", 0.2, "最大位置漂移退化"),
        ("post_event_overshoot_delta_m", 0.2, "停风后超调退化"),
        ("settling_time_delta_s", 2.0, "收敛时间退化"),
    ):
        value = item["metrics"][key]
        if value is not None and value > maximum + 1e-6:
            failed.append(description)
    if eb["settling_time_s"] is not None and ea["settling_time_s"] is None:
        failed.append("原生已收敛而候选未收敛")
    if active["motor_effort_mean"] > baseline["motor_effort_mean"] * 1.05:
        failed.append("电机平方和均值增加超过 5%")
    if active["allocation_failure_fraction"] > baseline["allocation_failure_fraction"] + 0.01:
        failed.append("分配器饱和比例增加超过 1 个百分点")
    if any(value > 0.3501 for value in ea["candidate_axis_max_m_s2"][:2]) or \
            ea["candidate_axis_max_m_s2"][2] > 1e-4 or \
            ea["candidate_max_correction_m_s2"] > 0.5001:
        failed.append("候选修正超过逐轴限制或 Z 非零")
    if case["disturbance"] == "wind" and ea["candidate_max_abs_before_window_end_m_s2"] > 1e-4:
        failed.append("固定风阶段候选非零")
    if failed:
        item.update(classification="FAIL", reasons=failed)
        return item
    if ea["settling_time_s"] is None and eb["settling_time_s"] is None:
        item["observations"].append("双方均未在记录窗口内达到 0.20 m 连续 2 s 收敛")
    if case["disturbance"] == "gust" and case["magnitude_m_s"] >= 6:
        benefit = item["metrics"]["event_xy_rmse_delta_m"]
        if benefit is None or benefit > -0.05:
            item["reasons"].append("强阵风事件改善未达到 0.05 m")
    item["classification"] = "PARTIAL" if item["reasons"] else "PASS"
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    matrix = json.loads((here / "robustness_matrix.json").read_text(encoding="utf-8"))
    cases = {case["id"]: case for case in matrix["cases"]}
    results = json.loads((args.run / "results.json").read_text(encoding="utf-8"))
    extended = json.loads((args.run / "extended.json").read_text(encoding="utf-8"))
    ext_by_key = {(item["id"], item["attempt"]): item for item in extended}
    latest = {pair["id"]: pair for pair in results}
    decisions = [classify(pair, ext_by_key[(pair["id"], pair["attempt"])], cases[pair["id"]])
                 for pair in latest.values()]
    missing = sorted(set(cases) - {item["id"] for item in decisions})
    worst = {}
    for key in ("event_xy_rmse_delta_m", "xy_error_p95_delta_m", "xy_excursion_max_delta_m",
                "post_event_overshoot_delta_m", "settling_time_delta_s"):
        values = [(item["metrics"][key], item["id"]) for item in decisions
                  if "metrics" in item and item["metrics"][key] is not None]
        worst[key] = {"value": max(values)[0], "id": max(values)[1]} if values else None
    overall = ("FAIL" if any(item["classification"] == "FAIL" for item in decisions) else
               "PARTIAL" if missing or any(item["classification"] == "PARTIAL" for item in decisions)
               else "PASS")
    output = {"overall": overall, "missing_ids": missing, "worst_case_degradation": worst,
              "cases": decisions}
    (args.run / "classification.json").write_text(json.dumps(output, indent=2, sort_keys=True) + "\n",
                                                     encoding="utf-8")


if __name__ == "__main__":
    main()
