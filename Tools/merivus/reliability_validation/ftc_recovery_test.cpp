/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
// Host-only regression tests. No uORB, transport, actuator, or hardware access.
// The fixture supplies ideal state observations; it is not a flight dynamics simulation.
#include "src/modules/ftc_recovery/FtcRecoveryController.hpp"
#include "src/modules/ftc_recovery/FtcRecoveryArbiter.hpp"

#include <cmath>
#include <cstdio>
#include <functional>
#include <stdexcept>
#include <string>

#define CHECK(condition) do { if (!(condition)) { \
	throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #condition); \
} } while (false)

using Controller = FtcRecoveryController;
using Arbiter = FtcRecoveryArbiter;

struct Fixture {
	Controller controller;
	Controller::Input in{};

	explicit Fixture(uint8_t z_counter = 0, uint8_t vz_counter = 0)
	{
		in.enabled = in.eligible = in.fresh = in.controllable = in.vertical_valid = in.position_valid = in.mode_allowed = true;
		in.now = in.position_timestamp = 1000000;
		in.z = -10.f;
		in.mode = 3;
		in.z_reset_counter = z_counter;
		in.vz_reset_counter = vz_counter;
		controller.update(0.02f, in);
		CHECK(controller.output().state == Controller::MONITORING);
		in.armed = true;
		in.landed = false;
		in.trigger = 1;
	}

	void tick(bool refresh_position = true)
	{
		in.now += 20000;
		if (refresh_position) { in.position_timestamp = in.now; }
		controller.update(0.02f, in);
	}

	void reach(uint8_t state)
	{
		for (unsigned i = 0; i < 900 && controller.output().state != state; ++i) { tick(); }
		CHECK(controller.output().state == state);
		CHECK(controller.output().candidate_valid);
	}

	void expect_reset_abort()
	{
		CHECK(controller.output().state == Controller::ABORTED);
		CHECK(!controller.output().candidate_valid);
		CHECK(!controller.output().reentry_ready);
		CHECK(controller.output().fallback_reason == Controller::VERTICAL_STATE_RESET);
		CHECK(Controller::requiresImmediateExit(controller.output().fallback_reason));
	}
};

// Synthetic message-boundary adapter: production Controller, production exit
// reason helper, and production Arbiter are executed. uORB scheduling is not modeled.
static Arbiter::Input arbitration_input(const Fixture &fixture)
{
	const auto &candidate = fixture.controller.output();
	Arbiter::Input in{};
	in.now = in.normal_timestamp = in.candidate_timestamp = fixture.in.now;
	in.enabled = in.flight_allowed = in.mode_matches = true;
	in.candidate_valid = candidate.candidate_valid && fixture.in.eligible;
	in.hard_exit = Controller::requiresImmediateExit(candidate.fallback_reason);
	in.reentry = candidate.state == Controller::CONTROL_REENTRY;
	in.requested_weight = in.reentry ? candidate.reentry_weight : 1.f;
	for (unsigned i = 0; i < 3; ++i) {
		in.normal[i] = fixture.in.normal_rates[i];
		in.candidate[i] = candidate.rate[i];
	}
	in.normal[3] = fixture.in.normal_thrust;
	in.candidate[3] = candidate.thrust;
	return in;
}

int main()
{
	unsigned passed = 0;
	unsigned failed = 0;
	auto run = [&](const std::string &name, const std::function<void()> &test) {
		try {
			test();
			++passed;
			std::printf("PASS %s\n", name.c_str());
		} catch (const std::exception &error) {
			++failed;
			std::printf("FAIL %s: %s\n", name.c_str(), error.what());
		}
	};

	run("disabled_controller_has_no_candidate", [] {
		Controller controller;
		Controller::Input in{};
		in.armed = true;
		in.landed = false;
		in.eligible = in.fresh = in.controllable = in.mode_allowed = true;
		in.trigger = 1;
		controller.update(0.02f, in);
		CHECK(controller.output().state == Controller::DISABLED);
		CHECK(!controller.output().candidate_valid);
	});

	run("ineligible_trigger_is_not_consumed", [] {
		Fixture f;
		f.in.eligible = false;
		for (unsigned i = 0; i < 100; ++i) {
			f.tick();
			CHECK(f.controller.output().state == Controller::MONITORING);
			CHECK(!f.controller.output().candidate_valid);
		}
		f.in.eligible = true;
		f.tick();
		CHECK(f.controller.output().state == Controller::RATE_DAMPING);
		CHECK(f.controller.output().candidate_valid);
	});

	run("nonfinite_trigger_is_not_consumed", [] {
		Fixture f;
		f.in.z = NAN;
		f.tick();
		CHECK(f.controller.output().state == Controller::MONITORING);
		CHECK(!f.controller.output().candidate_valid);
		f.in.z = -10.f;
		f.tick();
		CHECK(f.controller.output().candidate_valid);
	});

	for (uint8_t state : {Controller::DISABLED, Controller::MONITORING, Controller::DISTURBANCE_DETECTED}) {
		run("entry_altitude_boundaries_state_" + std::to_string(state), [state] {
			CHECK(!Controller::entryAltitudeAllowed(state, true, false, 0.f, true, -2.99f, 3.f));
			CHECK(Controller::entryAltitudeAllowed(state, true, false, 0.f, true, -3.f, 3.f));
			CHECK(Controller::entryAltitudeAllowed(state, true, true, 3.f, false, NAN, 3.f));
			CHECK(!Controller::entryAltitudeAllowed(state, true, true, 1.f, true, -10.f, 3.f));
		});
	}

	struct AltitudeCase {
		const char *name;
		bool fresh, range_valid;
		float range;
		bool z_valid;
		float z, minimum;
	};
	const AltitudeCase invalid_altitudes[] {
		{"missing", true, false, 0.f, false, 0.f, 3.f},
		{"stale", false, false, 0.f, true, -10.f, 3.f},
		{"nan_z", true, false, 0.f, true, NAN, 3.f},
		{"inf_z", true, false, 0.f, true, -INFINITY, 3.f},
		{"nan_range", true, true, NAN, true, -10.f, 3.f},
		{"inf_range", true, true, INFINITY, true, -10.f, 3.f},
		{"nan_threshold", true, false, 0.f, true, -10.f, NAN},
		{"inf_threshold", true, false, 0.f, true, -10.f, INFINITY},
	};
	for (const auto &value : invalid_altitudes) {
		run(std::string("invalid_altitude_") + value.name, [value] {
			Fixture f;
			for (unsigned i = 0; i < 3; ++i) {
				f.in.eligible = Controller::entryAltitudeAllowed(f.controller.output().state, value.fresh,
						value.range_valid, value.range, value.z_valid, value.z, value.minimum);
				f.tick();
				CHECK(f.controller.output().state == Controller::MONITORING);
				CHECK(!f.controller.output().candidate_valid);
			}
		});
	}

	run("low_altitude_cannot_bypass_gate_on_second_cycle", [] {
		Fixture f;
		f.in.z = -1.f;
		for (unsigned i = 0; i < 100; ++i) {
			f.in.eligible = Controller::entryAltitudeAllowed(f.controller.output().state, true, false, 0.f, true, f.in.z, 3.f);
			f.tick();
			CHECK(f.controller.output().state == Controller::MONITORING);
			CHECK(!f.controller.output().candidate_valid);
		}
		f.in.z = -3.f;
		f.in.eligible = Controller::entryAltitudeAllowed(f.controller.output().state, true, false, 0.f, true, f.in.z, 3.f);
		f.tick();
		CHECK(f.controller.output().candidate_valid);
	});

	run("height_gate_is_entry_only", [] {
		CHECK(Controller::entryAltitudeAllowed(Controller::RATE_DAMPING, false, false, NAN, false, NAN, NAN));
		CHECK(Controller::entryAltitudeAllowed(Controller::EMERGENCY_LAND, true, true, 1.f, true, -1.f, 3.f));
	});

	run("nonzero_counter_first_sample_establishes_baseline", [] {
		Fixture f(100, 200);
		f.tick();
		CHECK(f.controller.output().candidate_valid);
		CHECK(f.controller.output().fallback_reason == 0);
	});

	run("counter_reset_in_monitoring_uses_new_entry_frame", [] {
		Fixture f;
		f.in.trigger = 0;
		f.in.z_reset_counter = 4;
		f.in.vz_reset_counter = 7;
		f.in.z = -5.f;
		f.tick();
		CHECK(f.controller.output().state == Controller::MONITORING);
		f.in.trigger = 1;
		f.tick();
		CHECK(f.controller.output().candidate_valid);
		CHECK(f.controller.output().altitude_reference == -5.f);
		CHECK(f.controller.output().fallback_reason == 0);
	});

	run("counter_change_same_cycle_as_initial_entry_is_not_old_target", [] {
		Fixture f;
		f.in.z_reset_counter = 3;
		f.in.z = -6.f;
		f.tick();
		CHECK(f.controller.output().candidate_valid);
		CHECK(f.controller.output().altitude_reference == -6.f);
	});

	run("repeated_position_sample_does_not_repeat_reset", [] {
		Fixture f;
		f.tick();
		f.tick(false);
		CHECK(f.controller.output().candidate_valid);
		CHECK(f.controller.output().fallback_reason == 0);
	});

	for (const std::string kind : {"z", "vz", "both", "missed_z", "missed_vz", "z_wrap", "vz_wrap", "time_backwards", "same_timestamp_reset"}) {
		run("active_reset_" + kind, [kind] {
			Fixture f(kind == "z_wrap" ? 255 : 0, kind == "vz_wrap" ? 255 : 0);
			f.reach(Controller::ALTITUDE_STABILIZATION);
			if (kind == "z" || kind == "both" || kind == "same_timestamp_reset") { ++f.in.z_reset_counter; f.in.z += 5.f; }
			if (kind == "vz" || kind == "both") { ++f.in.vz_reset_counter; f.in.vz += 1.f; }
			if (kind == "missed_z") { f.in.z_reset_counter += 3; f.in.z += 5.f; }
			if (kind == "missed_vz") { f.in.vz_reset_counter += 3; f.in.vz += 1.f; }
			if (kind == "z_wrap") { f.in.z_reset_counter = 0; f.in.z += 5.f; }
			if (kind == "vz_wrap") { f.in.vz_reset_counter = 0; f.in.vz += 1.f; }
			if (kind == "time_backwards") { f.in.position_timestamp -= 1; }
			f.tick(kind != "time_backwards" && kind != "same_timestamp_reset");
			f.expect_reset_abort();
			for (unsigned i = 0; i < 10; ++i) { f.tick(); f.expect_reset_abort(); }
		});
	}

	for (uint8_t state : {Controller::RATE_DAMPING, Controller::THRUST_VECTOR_RECOVERY, Controller::ATTITUDE_RECOVERY,
			Controller::VERTICAL_SPEED_RECOVERY, Controller::ALTITUDE_STABILIZATION, Controller::CONTROL_REENTRY,
			Controller::EMERGENCY_LAND}) {
		run("reset_cancels_each_active_phase_" + std::to_string(state), [state] {
			Fixture f;
			if (state == Controller::EMERGENCY_LAND) { f.in.position_valid = false; }
			f.reach(state);
			++f.in.z_reset_counter;
			f.in.z += 5.f;
			f.tick();
			f.expect_reset_abort();
		});
	}

	run("complete_sequence_releases_arbiter_and_does_not_retrigger", [] {
		Fixture f;
		Arbiter arbiter;
		bool seen[12] {};
		for (unsigned i = 0; i < 800; ++i) {
			f.in.arbitration_weight = arbiter.weight;
			f.tick();
			const auto &candidate = f.controller.output();
			seen[candidate.state] = true;
			CHECK(candidate.state != Controller::FAILED && candidate.state != Controller::ABORTED);
			arbiter.update(0.02f, arbitration_input(f));
			for (float value : arbiter.output) { CHECK(std::isfinite(value)); }
		}
		for (uint8_t state : {Controller::RATE_DAMPING, Controller::THRUST_VECTOR_RECOVERY, Controller::ATTITUDE_RECOVERY,
				Controller::VERTICAL_SPEED_RECOVERY, Controller::ALTITUDE_STABILIZATION, Controller::CONTROL_REENTRY}) {
			CHECK(seen[state]);
		}
		CHECK(f.controller.output().state == Controller::MONITORING);
		CHECK(!f.controller.output().candidate_valid);
		CHECK(!arbiter.active && arbiter.weight == 0.f);
		for (unsigned i = 0; i < 3; ++i) { CHECK(arbiter.output[i] == f.in.normal_rates[i]); }
		CHECK(arbiter.output[3] == f.in.normal_thrust);
		f.in.trigger = 0;
		f.tick();
		f.in.trigger = 1;
		f.tick();
		CHECK(f.controller.output().candidate_valid);
	});

	run("reset_hard_exit_removes_held_candidate_in_same_arbiter_update", [] {
		Fixture f;
		Arbiter arbiter;
		f.in.rates[0] = 1.f;
		for (unsigned i = 0; i < 70; ++i) {
			f.tick();
			arbiter.update(0.02f, arbitration_input(f));
		}
		CHECK(arbiter.active && arbiter.weight > 0.99f);
		CHECK(std::fabs(arbiter.output[0] - f.in.normal_rates[0]) > 0.1f);
		++f.in.vz_reset_counter;
		f.tick();
		f.expect_reset_abort();
		arbiter.update(0.02f, arbitration_input(f));
		CHECK(!arbiter.active && arbiter.weight == 0.f);
		for (unsigned i = 0; i < 3; ++i) { CHECK(arbiter.output[i] == f.in.normal_rates[i]); }
		CHECK(arbiter.output[3] == f.in.normal_thrust);
	});

	run("normal_invalid_candidate_uses_existing_gradual_exit", [] {
		Fixture f;
		Arbiter arbiter;
		f.in.rates[0] = 1.f;
		for (unsigned i = 0; i < 70; ++i) { f.tick(); arbiter.update(0.02f, arbitration_input(f)); }
		CHECK(arbiter.active);
		Arbiter::Input in = arbitration_input(f);
		in.candidate_valid = false;
		arbiter.update(0.02f, in);
		CHECK(arbiter.active && arbiter.weight > 0.9f && arbiter.weight < 1.f);
		for (unsigned i = 0; i < 40; ++i) {
			in.now += 20000;
			in.normal_timestamp = in.now;
			arbiter.update(0.02f, in);
		}
		CHECK(!arbiter.active && arbiter.weight == 0.f);
	});

	run("disarm_after_reset_allows_clean_new_flight", [] {
		Fixture f;
		f.tick();
		++f.in.z_reset_counter;
		f.tick();
		f.expect_reset_abort();
		f.in.armed = false;
		f.in.landed = true;
		f.tick();
		CHECK(f.controller.output().state == Controller::MONITORING);
		f.in.armed = true;
		f.in.landed = false;
		f.tick();
		CHECK(f.controller.output().candidate_valid);
		CHECK(f.controller.output().fallback_reason == 0);
	});

	for (const std::string kind : {"failsafe", "stale_state", "mode_change", "disarm", "landed", "authority_loss"}) {
		run("normal_safety_exit_" + kind, [kind] {
			Fixture f;
			f.tick();
			CHECK(f.controller.output().candidate_valid);
			if (kind == "failsafe") { f.in.failsafe = true; }
			if (kind == "stale_state") { f.in.fresh = false; }
			if (kind == "mode_change") { ++f.in.mode; }
			if (kind == "disarm") { f.in.armed = false; }
			if (kind == "landed") { f.in.landed = true; }
			if (kind == "authority_loss") { f.in.controllable = false; }
			f.tick();
			CHECK(!f.controller.output().candidate_valid);
			CHECK(f.controller.output().state == (kind == "authority_loss" ? Controller::FAILED : Controller::ABORTED));
		});
	}

	std::printf("SUMMARY passed=%u failed=%u\n", passed, failed);
	return failed == 0 ? 0 : 1;
}
