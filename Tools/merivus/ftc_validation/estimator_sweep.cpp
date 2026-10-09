#include "src/modules/motor_health_monitor/MotorEffectivenessEstimator.hpp"
#include "src/modules/motor_health_monitor/CommandAlignment.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {
struct Scenario {
    std::string name;
    float target{1.f};
    unsigned motor{0};
    unsigned actual_delay{40}, assumed_delay{40};
    std::string pattern{"step"};
};
float command(float t, unsigned i, const std::string &pattern)
{
    if (pattern == "hover") { return .5f; }
    if (pattern == "collective") { return .5f + .12f * std::sin(t * 1.2f); }
    if (pattern == "single_axis") { return .5f + (i == 0 || i == 3 ? .12f : -.12f) * std::sin(t * 1.2f); }
    if (pattern == "collective_dominant") { return .5f + .18f * std::sin(t * 1.2f) + .0001f * std::sin(t * (1.7f+i)); }
    return .5f + .12f * std::sin(t * (1.2f + i * .7f));
}
float truth(float t, const Scenario &s)
{
    if (t < 35.f) { return 1.f; }
    if (s.pattern == "ramp") { return 1.f - std::min((t-35.f)/10.f, 1.f) * (1.f-s.target); }
    if (s.pattern == "intermittent") { return std::fmod(t-35.f, 8.f) < 4.f ? s.target : 1.f; }
    return s.target;
}
void run(const Scenario &s)
{
    MotorEffectivenessEstimator estimator;
    MotorEffectivenessEstimator::Configuration config;
    CommandAlignment buffer;
    constexpr float dt = .02f;
    float absolute_sum = 0.f, other_max = 0.f, sigma_sum = 0.f, confidence_sum = 0.f;
    float detected = -1.f, stable = -1.f, stable_elapsed = 0.f;
    unsigned measured = 0, valid_count = 0, observable_count = 0;
    bool learned_before = false;
    for (unsigned k = 0; k < 4500; ++k) {
        const float t = k * dt;
        const auto pattern = t >= 35.f && s.pattern == "history_hover" ? std::string("hover") : s.pattern;
        float control[12] {};
        for (unsigned i = 0; i < 4; ++i) { control[i] = command(t, i, pattern); }
        const uint64_t timestamp = 1000000 + k * 20000;
        buffer.push(timestamp, control);
        MotorEffectivenessEstimator::Input input{};
        input.timestamp = timestamp;
        input.motor_count = 4;
        input.valid = true;
        input.aligned = buffer.sample(timestamp, s.assumed_delay*1000, input.control);
        for (unsigned i = 0; i < 4; ++i) {
            input.geometry[0][i] = i == 0 || i == 3 ? .2f : -.2f;
            input.geometry[1][i] = i < 2 ? .2f : -.2f;
            input.geometry[2][i] = i % 2 ? .04f : -.04f;
            const float response_t = t - s.actual_delay * .001f;
            const float response = command(response_t, i, pattern) * (i == s.motor ? truth(response_t, s) : 1.f);
            for (unsigned a = 0; a < 3; ++a) { input.angular_acceleration[a] += 40.f * input.geometry[a][i] * response; }
        }
        estimator.update(dt, input, config);
        const auto &out = estimator.output();
        if (k == 1749) { learned_before = out.baseline_learned; }
        const float error = std::fabs(out.effectiveness[s.motor] - truth(t, s));
        if (t > 35.f && out.estimate_valid && out.effectiveness[s.motor] < .975f && detected < 0.f) { detected = t-35.f; }
        stable_elapsed = t > 35.f && out.estimate_valid && error < .05f ? stable_elapsed+dt : 0.f;
        if (stable_elapsed >= 2.f && stable < 0.f) { stable = t-35.f-2.f; }
        if (t >= 80.f) {
            ++measured;
            absolute_sum += error;
            sigma_sum += out.uncertainty[s.motor];
            confidence_sum += out.confidence[s.motor];
            valid_count += out.estimate_valid;
            observable_count += out.current_observable;
            for (unsigned i = 0; i < 4; ++i) {
                if (i != s.motor) { other_max = std::max(other_max, std::fabs(1.f-out.effectiveness[i])); }
            }
        }
    }
    const auto &out = estimator.output();
    std::printf("%s,%u,%.2f,%u,%u,%.6f,%.6f,%.6f,%.3f,%.3f,%.6f,%.6f,%.3f,%u,%u,%.3f,%.3f,%.3f\n",
                s.name.c_str(),s.motor+1,(double)s.target,s.actual_delay,s.assumed_delay,
                (double)out.effectiveness[s.motor],(double)(absolute_sum/measured),(double)other_max,
                (double)detected,(double)stable,(double)(sigma_sum/measured),(double)(confidence_sum/measured),
                (double)out.estimate_age,(unsigned)out.state,(unsigned)learned_before,
                (double)valid_count/measured,(double)observable_count/measured,(double)out.condition_number);
}
}
int main()
{
    std::puts("scenario,motor,truth,actual_delay_ms,assumed_delay_ms,final_lambda,mae_last10s,other_motor_max_error,detection_s,stable_s,sigma_mean,confidence_mean,age_final,state_final,baseline_at35s,valid_fraction,observable_fraction,condition_final");
    for (float lambda : {1.f,.95f,.9f,.8f,.7f,.5f}) {
        for (unsigned motor=0; motor<4; ++motor) { run({"truth",lambda,motor,40,40,"step"}); }
    }
    for (const char *pattern : {"hover","collective","collective_dominant","single_axis","history_hover","ramp","intermittent"}) {
        run({pattern, std::string(pattern)=="ramp" || std::string(pattern)=="intermittent" ? .7f : 1.f,0,40,40,pattern});
    }
    for (unsigned delay : {0u,20u,40u,60u,80u}) {
        run({"matched_delay",.7f,0,delay,delay,"step"});
        run({"wrong_delay",.7f,0,40,delay,"step"});
    }
    return 0;
}
