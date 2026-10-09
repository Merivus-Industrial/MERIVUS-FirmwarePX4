/****************************************************************************
 * Copyright (c) 2026 Merivus Industrial. All rights reserved.
 ****************************************************************************/
#pragma once
#include <stdint.h>
#include <string.h>

class CommandAlignment
{
public:
	static constexpr unsigned Capacity = 32;
	static constexpr unsigned Motors = 12;
	void reset() { _count = _next = 0; _latest = 0; }
	void push(uint64_t timestamp, const float control[Motors])
	{
		if (!timestamp || timestamp == _latest) { return; }
		if (timestamp < _latest) { reset(); }
		_latest = timestamp;
		_frames[_next].timestamp = timestamp;
		memcpy(_frames[_next].control, control, sizeof(_frames[_next].control));
		_next = (_next + 1) % Capacity;
		if (_count < Capacity) { ++_count; }
	}
	bool sample(uint64_t response_timestamp, uint32_t delay_us, float control[Motors]) const
	{
		if (response_timestamp <= delay_us) { return false; }
		const uint64_t target = response_timestamp - delay_us;
		const Frame *before = nullptr;
		for (unsigned i = 0; i < _count; ++i) {
			if (_frames[i].timestamp <= target && (!before || _frames[i].timestamp > before->timestamp)) { before = &_frames[i]; }
		}
		// Actuator commands are held until the next output; never interpolate a future command into an earlier response.
		if (!before || target - before->timestamp > 40000) { return false; }
		memcpy(control, before->control, sizeof(before->control));
		return true;
	}
private:
	struct Frame { uint64_t timestamp{0}; float control[Motors] {}; };
	Frame _frames[Capacity] {};
	unsigned _count{0}, _next{0};
	uint64_t _latest{0};
};
