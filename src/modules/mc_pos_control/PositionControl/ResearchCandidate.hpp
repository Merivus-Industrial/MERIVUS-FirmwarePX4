#pragma once

#include <cmath>
#include <cstdint>

#include <matrix/matrix/math.hpp>

namespace afcr
{

enum class Op : uint8_t {
	VelocityError,
	AccelerationResidual,
	VelocityGain,
	LowpassAcceleration,
	AccelerationGain,
	AddAcceleration,
	VelocityGate,
	LimitAcceleration
};

struct Node {
	Op op;
	int8_t input_a;
	int8_t input_b;
	float x;
	float y;
	float z;
};

#include "ResearchCandidateSpec.hpp"

class Candidate
{
public:
	using Vector3f = matrix::Vector3f;
	Candidate() { reset(); }

	struct Result {
		Vector3f acceleration;
		bool valid;
	};

	void reset()
	{
		for (unsigned i = 0; i < node_count; ++i) {
			_memory[i].setZero();
		}

		_last_applied.setZero();
		_has_previous = false;
	}

	Result update(const Vector3f &velocity_error, const Vector3f &measured_acceleration,
		      const Vector3f &baseline_acceleration, float dt, bool apply)
	{
		if (!velocity_error.isAllFinite() || !measured_acceleration.isAllFinite()
		    || !baseline_acceleration.isAllFinite() || !std::isfinite(dt)
		    || dt < 0.002f || dt > 0.04f) {
			reset();
			return {baseline_acceleration, false};
		}

		const Vector3f residual = _has_previous ? measured_acceleration - _last_applied : Vector3f(0.f, 0.f, 0.f);

		for (unsigned i = 0; i < node_count; ++i) {
			const Node &node = nodes[i];
			Vector3f value(0.f, 0.f, 0.f);

			switch (node.op) {
			case Op::VelocityError:
				value = velocity_error;
				break;

			case Op::AccelerationResidual:
				value = residual;
				break;

			case Op::VelocityGain:
			case Op::AccelerationGain:
				value = _values[node.input_a].emult(Vector3f(node.x, node.y, node.z));
				break;

			case Op::LowpassAcceleration: {
				const float alpha = dt / (node.x + dt);
				_memory[i] += (_values[node.input_a] - _memory[i]) * alpha;
				value = _memory[i];
				break;
			}

			case Op::AddAcceleration:
				value = _values[node.input_a] + _values[node.input_b];
				break;

			case Op::VelocityGate: {
				const Vector3f &velocity = _values[node.input_a];
				const float horizontal_error = sqrtf(velocity(0) * velocity(0) + velocity(1) * velocity(1));
				const float weight = fmaxf(0.f, fminf(1.f, (horizontal_error - node.x) / (node.y - node.x)));
				value = _values[node.input_b] * weight;
				break;
			}

			case Op::LimitAcceleration:
				for (unsigned axis = 0; axis < 3; ++axis) {
					const float limit = axis == 0 ? node.x : (axis == 1 ? node.y : node.z);
					value(axis) = fmaxf(-limit, fminf(limit, _values[node.input_a](axis)));
				}

				break;
			}

			_values[i] = value;
		}

		const Vector3f correction = _values[output_node];
		const Vector3f candidate = baseline_acceleration + correction;

		if (!candidate.isAllFinite()) {
			reset();
			return {baseline_acceleration, false};
		}

		for (unsigned axis = 0; axis < 3; ++axis) {
			if (fabsf(correction(axis)) > maximum_correction) {
				reset();
				return {baseline_acceleration, false};
			}
		}

		_last_applied = apply ? candidate : baseline_acceleration;
		_has_previous = true;
		return {candidate, true};
	}

private:
	Vector3f _values[node_count]{};
	Vector3f _memory[node_count]{};
	Vector3f _last_applied{0.f, 0.f, 0.f};
	bool _has_previous{false};
};

} // namespace afcr
