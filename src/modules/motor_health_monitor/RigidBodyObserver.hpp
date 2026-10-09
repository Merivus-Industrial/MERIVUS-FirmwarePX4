/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
#pragma once
#include <math.h>

// Absolute mass requires an independently calibrated thrust scale. Neither lambda nor a normalized mixer provides it.
class RigidBodyObserver
{
public:
	void reset() { _mass = NAN; _elapsed = _age = 0.f; }
	void update(float dt, float thrust_newton, float specific_force, bool observable)
	{
		_age += dt;
		if (!observable || !isfinite(thrust_newton) || thrust_newton <= 0.f
		    || !isfinite(specific_force) || specific_force < 3.f) { return; }
		const float sample = thrust_newton / specific_force;
		if (sample < 0.05f || sample > 100.f) { return; }
		_mass = isfinite(_mass) ? _mass + dt / (5.f + dt) * (sample - _mass) : sample;
		_elapsed += dt;
		_age = 0.f;
	}
	bool valid() const { return _elapsed >= 5.f && _age < 2.f && isfinite(_mass); }
	float mass() const { return _mass; }
	float age() const { return _age; }
private:
	float _mass{NAN}, _elapsed{0.f}, _age{0.f};
};
