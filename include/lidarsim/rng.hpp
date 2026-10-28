// Counter-based random numbers.
//
// Each beam gets its own generator seeded from (seed, frame, channel, azimuth). A beam's noise
// therefore depends only on which beam it is, never on which thread ran it or in what order,
// so a scan is bit-identical for any thread count.
#pragma once

#include <cmath>
#include <cstdint>

namespace lidarsim {

class Rng {
public:
    Rng(uint64_t seed, uint64_t a, uint64_t b, uint64_t c) {
        state_ = mix(seed);
        state_ = mix(state_ ^ a);
        state_ = mix(state_ ^ b);
        state_ = mix(state_ ^ c);
    }

    // splitmix64 step.
    uint64_t next() {
        state_ += 0x9E3779B97F4A7C15ull;
        return mix(state_);
    }

    // Uniform in [0, 1) using the top 53 bits (a double's mantissa).
    double uniform() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
    double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

    // Standard normal via Box–Muller (uses two uniforms, returns one sample; no hidden state).
    double normal() {
        const double u1 = 1.0 - uniform();  // in (0, 1], so log() is finite
        const double u2 = uniform();
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * 3.14159265358979323846 * u2);
    }

private:
    static uint64_t mix(uint64_t z) {
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    uint64_t state_;
};

}  // namespace lidarsim
