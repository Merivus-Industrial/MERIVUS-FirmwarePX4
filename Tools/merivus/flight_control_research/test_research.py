import copy
import hashlib
import json
from pathlib import Path
import unittest

import candidate
import classify_robustness
import cycle
import evaluate
import research
import run_sitl
import shadow


class CandidateTest(unittest.TestCase):
    def setUp(self):
        self.spec = json.loads(Path(__file__).with_name("candidate.json").read_text(encoding="utf-8"))

    def test_checked_in_header_matches_graph(self):
        header = Path(__file__).parents[3] / "src/modules/mc_pos_control/PositionControl/ResearchCandidateSpec.hpp"
        self.assertEqual(header.read_text(encoding="utf-8"), candidate.generate(self.spec))
        self.assertIn("static constexpr float maximum_correction = 0.5f;", candidate.generate(self.spec))

    def test_rejects_unit_mismatch_and_unbounded_output(self):
        wrong = copy.deepcopy(self.spec)
        wrong["nodes"][5]["op"] = "velocity_gain"
        with self.assertRaisesRegex(ValueError, "expected velocity"):
            candidate.validate(wrong)
        wrong = copy.deepcopy(self.spec)
        wrong["output"] = "combined"
        with self.assertRaisesRegex(ValueError, "final limit"):
            candidate.validate(wrong)

    def test_velocity_gate_requires_ordered_finite_thresholds_and_units(self):
        wrong = copy.deepcopy(self.spec)
        wrong["nodes"][-2]["threshold_m_s"] = [0.25, 0.05]
        with self.assertRaisesRegex(ValueError, "upper threshold"):
            candidate.validate(wrong)
        wrong["nodes"][-2]["threshold_m_s"] = [0.05, float("nan")]
        with self.assertRaisesRegex(ValueError, "finite number"):
            candidate.validate(wrong)
        wrong = copy.deepcopy(self.spec)
        wrong["nodes"][-2]["left"] = "combined"
        with self.assertRaisesRegex(ValueError, "expected velocity"):
            candidate.validate(wrong)


def telemetry(error=0.1, saturated=False):
    times = [i * 20_000 for i in range(1000)]
    truth = {"timestamp": times, "x": [error] * 1000, "y": [0.0] * 1000,
             "z": [-5.0 + error] * 1000}
    reference = {"timestamp": times[::5], "x": [0.0] * 200, "y": [0.0] * 200,
                 "z": [-5.0] * 200}
    allocator = {"timestamp": times[::10], "thrust_setpoint_achieved": [not saturated] * 100,
                 "torque_setpoint_achieved": [True] * 100}
    motors = {"timestamp": times[::5]}
    for axis in range(4):
        motors[f"control[{axis}]"] = [0.5] * 200
    status = {"timestamp": times[::10], "arming_state": [2] * 100, "nav_state": [4] * 100}
    return truth, reference, allocator, motors, status


class EvaluationTest(unittest.TestCase):
    def test_candidate_hard_gate_failure_is_not_excused_by_baseline_failure(self):
        metric = {"hard_gate_passed": False, "motor_effort_mean": 1.0,
                  "allocation_failure_fraction": 0.0, "event_xy_rmse_m": 0.8}
        pair = {"id": "w12_135_g2", "trials": {
            mode: {"metrics": {"gust_hover": dict(metric)}, "ulog": mode + ".ulg"}
            for mode in ("active", "off")}}
        tail = {"xy_error_p95_m": 1.0, "xy_excursion_max_m": 2.1,
                "post_event_overshoot_m": 1.0, "settling_time_s": None,
                "candidate_activation_fraction": 0.2, "candidate_max_correction_m_s2": 0.4,
                "candidate_max_abs_before_window_end_m_s2": 0.35,
                "candidate_axis_max_m_s2": [0.35, 0.35, 0.0],
                "candidate_limit_incidence": 0.0, "candidate_slope_p95_m_s3": 0.2}
        extended = {"trials": {mode: dict(tail) for mode in ("active", "off")}}
        case = {"disturbance": "gust", "magnitude_m_s": 12}
        verdict = classify_robustness.classify(pair, extended, case)
        self.assertEqual(verdict["classification"], "FAIL")
        self.assertIn("候选安全硬门失败", verdict["reasons"])

    def test_trial_order_counterbalances_odd_and_even_seeds(self):
        self.assertEqual(cycle.mode_order_for_seed(11), ("active", "off"))
        self.assertEqual(cycle.mode_order_for_seed(12), ("off", "active"))

    def test_wind_scenarios_configure_the_gazebo_plugin(self):
        source = (Path(__file__).parents[3] /
                  "Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/windy.world")
        plugin = run_sitl.configure_wind_world(source, "gust", 1).find(".//plugin[@name='wind_plugin']")
        self.assertEqual(plugin.findtext("windVelocityMean"), "0")
        self.assertEqual(plugin.findtext("windGustStart"), "46")
        self.assertEqual(plugin.findtext("windGustDuration"), "5")
        self.assertEqual(plugin.findtext("windGustVelocityMean"), "8")
        self.assertEqual(plugin.findtext("windGustDirectionMean"), "1 0 0")
        plugin = run_sitl.configure_wind_world(source, "wind", 2).find(".//plugin[@name='wind_plugin']")
        self.assertEqual(plugin.findtext("windVelocityMean"), "4.0")
        self.assertEqual(plugin.findtext("windDirectionMean"), "0 1 0")

    def test_robustness_matrix_covers_frozen_axes(self):
        root = Path(__file__).parent
        matrix = json.loads((root / "robustness_matrix.json").read_text(encoding="utf-8"))
        self.assertEqual(matrix["candidate_sha256"],
                         hashlib.sha256((root / "candidate.json").read_bytes()).hexdigest())
        cases = matrix["cases"]
        self.assertEqual(len({case["id"] for case in cases}), len(cases))
        self.assertEqual({case["magnitude_m_s"] for case in cases if case["disturbance"] == "gust"},
                         {2, 4, 6, 8, 10, 12})
        self.assertTrue({"1 0 0", "-1 0 0", "0 1 0", "0 -1 0",
                         "0.70710678 0.70710678 0", "-0.70710678 0.70710678 0"}.issubset(
                             {case["direction"] for case in cases}))
        self.assertEqual({case["duration_s"] for case in cases if case["disturbance"] == "gust"},
                         {0.5, 1, 2, 5})
        self.assertEqual({case["disturbance"] for case in cases},
                         {"gust", "wind", "periodic", "randomized_gust"})

    def test_nondefault_wind_case_is_written_to_world(self):
        source = (Path(__file__).parents[3] /
                  "Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/windy.world")
        case = {"magnitude_m_s": 12, "direction": "-1 0 0", "duration_s": 0.5}
        plugin = run_sitl.configure_wind_world(source, "gust", 8, case).find(
            ".//plugin[@name='wind_plugin']")
        self.assertEqual(plugin.findtext("windGustVelocityMean"), "12")
        self.assertEqual(plugin.findtext("windGustDirectionMean"), "-1 0 0")
        self.assertEqual(plugin.findtext("windGustDuration"), "0.5")

    def test_takeoff_gate_uses_relative_height_and_stationary_vertical_motion(self):
        class Position:
            z = -1.7
            vz = 0.05

        self.assertTrue(run_sitl.hover_reached(Position(), 0.0))
        Position.z = -0.8
        self.assertFalse(run_sitl.hover_reached(Position(), 0.0))
        Position.z = -1.7
        Position.vz = -0.5
        self.assertFalse(run_sitl.hover_reached(Position(), 0.0))

    def test_truth_error_and_allocation_gate(self):
        good = evaluate.score_arrays(*telemetry(), 0, 20_000_000)
        self.assertAlmostEqual(good["xy_rmse_m"], 0.1)
        self.assertAlmostEqual(good["z_rmse_m"], 0.1)
        self.assertAlmostEqual(good["station_xy_rmse_m"], 0.0)
        self.assertAlmostEqual(good["station_z_rmse_m"], 0.0)
        self.assertTrue(good["hard_gate_passed"])
        bad = evaluate.score_arrays(*telemetry(saturated=True), 0, 20_000_000)
        self.assertFalse(bad["hard_gate_passed"])

    def test_requires_complete_truth(self):
        truth, reference, allocator, motors, status = telemetry()
        truth["timestamp"] = truth["timestamp"][:10]
        with self.assertRaisesRegex(ValueError, "insufficient"):
            evaluate.score_arrays(truth, reference, allocator, motors, status, 0, 20_000_000)

    def test_rejects_global_groundtruth_without_centimetre_resolution(self):
        data = telemetry()
        data[0]["x"] = [5_286_006.0] * 1000
        with self.assertRaisesRegex(evaluate.InvalidTruthFrame, "coordinate frame"):
            evaluate.score_arrays(*data, 0, 20_000_000)

    def test_stationkeeping_metric_uses_initial_truth_anchor(self):
        data = telemetry(error=0.0)
        data[0]["x"] = [0.0] * 100 + [0.2] * 900
        metrics = evaluate.score_arrays(*data, 0, 20_000_000)
        self.assertGreater(metrics["station_xy_rmse_m"], 0.18)
        self.assertAlmostEqual(metrics["station_z_rmse_m"], 0.0)
        event = evaluate.score_arrays(*data, 0, 20_000_000, 5_000_000, 10_000_000)
        self.assertAlmostEqual(event["event_xy_rmse_m"], 0.2)
        self.assertAlmostEqual(event["recovery_xy_rmse_m"], 0.2)

    def test_wrong_flight_mode_fails(self):
        data = telemetry()
        data[-1]["nav_state"] = [0] * 100
        self.assertFalse(evaluate.score_arrays(*data, 0, 20_000_000)["hard_gate_passed"])

    def test_comparison_checks_regression_and_stress(self):
        normal = {"hard_gate_passed": True, "station_xy_rmse_m": 0.1, "station_z_rmse_m": 0.1,
                  "motor_effort_mean": 1.0}
        stress = dict(normal, station_xy_rmse_m=0.4)
        gust = dict(normal, event_xy_rmse_m=0.4, event_z_rmse_m=0.2,
                    recovery_xy_rmse_m=0.3)
        baseline = {"normal_hover": normal, "wind_hover": stress,
                    "payload_hover": stress, "gust_hover": gust}
        candidate_result = {"normal_hover": dict(normal), "wind_hover": dict(stress),
                            "payload_hover": dict(stress),
                            "gust_hover": dict(gust, event_xy_rmse_m=0.3,
                                               recovery_xy_rmse_m=0.2)}
        quiet = {name: {"scored_samples": 1000, "max_abs_m_s2": 0.0}
                 for name in ("normal_hover", "wind_hover", "payload_hover")}
        self.assertTrue(evaluate.compare(baseline, candidate_result, quiet)["retain"])
        quiet["wind_hover"]["max_abs_m_s2"] = 0.01
        self.assertFalse(evaluate.compare(baseline, candidate_result, quiet)["retain"])

    def test_gust_improvement_must_be_in_the_event_window(self):
        normal = {"hard_gate_passed": True, "station_xy_rmse_m": 0.1,
                  "station_z_rmse_m": 0.1, "motor_effort_mean": 1.0}
        wind = dict(normal, station_xy_rmse_m=0.4)
        gust = dict(normal, event_xy_rmse_m=0.4, event_z_rmse_m=0.2,
                    recovery_xy_rmse_m=0.3)
        baseline = {"normal_hover": normal, "wind_hover": wind,
                    "payload_hover": dict(wind), "gust_hover": gust}
        candidate_result = {"normal_hover": dict(normal),
                            "wind_hover": dict(wind, station_xy_rmse_m=0.3),
                            "payload_hover": dict(wind),
                            "gust_hover": dict(gust)}
        quiet = {name: {"scored_samples": 1000, "max_abs_m_s2": 0.0}
                 for name in ("normal_hover", "wind_hover", "payload_hover")}
        self.assertFalse(evaluate.compare(baseline, candidate_result, quiet)["retain"])
        candidate_result["gust_hover"]["event_xy_rmse_m"] = 0.3
        candidate_result["gust_hover"]["recovery_xy_rmse_m"] = 0.2
        self.assertTrue(evaluate.compare(baseline, candidate_result, quiet)["retain"])


class ResearchTest(unittest.TestCase):
    def test_metadata_does_not_claim_performance(self):
        work = {"DOI": "10.1234/EXAMPLE", "title": ["A control paper"], "publisher": "Example",
                "published": {"date-parts": [[2024, 2]]}}
        record = research.normalize_work(work, "INDI", "2026-09-28T00:00:00Z")
        self.assertEqual(record["review_status"], "METADATA_ONLY")
        self.assertEqual(record["doi"], "10.1234/example")
        self.assertNotIn("performance", record)


class ShadowTest(unittest.TestCase):
    def test_quiescence_checks_pre_window_correction_and_logging(self):
        name = b"AFCR_DA\0\0\0"
        data = {"timestamp": [index * 20_000 for index in range(1000)],
                "x": [0.0] * 1000, "y": [0.0] * 1000, "z": [0.0] * 1000}
        for index, value in enumerate(name):
            data[f"name[{index}]"] = [value] * 1000
        window = {"start_s": 5, "end_s": 20}
        self.assertEqual(shadow.verify_quiescent_samples(data, window)["scored_samples"], 750)
        data["x"][10] = 0.01
        with self.assertRaisesRegex(ValueError, "active outside gust"):
            shadow.verify_quiescent_samples(data, window)

    def test_correction_is_logged_and_bounded(self):
        name = b"AFCR_DA\0\0\0"
        data = {"timestamp": [index * 20_000 for index in range(600)],
                "x": [0.1] * 600, "y": [0.2] * 600, "z": [0.0] * 600}
        for index, value in enumerate(name):
            data[f"name[{index}]"] = [value] * 600
        spec = json.loads(Path(__file__).with_name("candidate.json").read_text(encoding="utf-8"))
        result = shadow.verify_samples(data, {"start_s": 0, "end_s": 15}, spec)
        self.assertEqual(result["samples"], 600)
        self.assertAlmostEqual(result["max_abs_m_s2"][1], 0.2)
        data["y"][0] = 0.5
        with self.assertRaisesRegex(ValueError, "exceeded"):
            shadow.verify_samples(data, {"start_s": 0, "end_s": 15}, spec)


if __name__ == "__main__":
    unittest.main()
