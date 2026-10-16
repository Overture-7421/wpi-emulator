#pragma once

// Deterministic RNG (splitmix64). All randomness in the simulation (shot dispersion, spawn jitter)
// MUST come from SimContext::rng() so runs are reproducible from RobotConfig::sim.seed.

#include <cstdint>

namespace wpiemu::sdk {

class Rng {
public:
    explicit Rng(std::uint64_t seed = 0x9E3779B97F4A7C15ull) : state_(seed) {}

    void seed(std::uint64_t s) { state_ = s; }
    std::uint64_t state() const { return state_; }

    std::uint64_t nextU64() {
        std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    // Uniform in [0, 1).
    double nextDouble() { return static_cast<double>(nextU64() >> 11) * (1.0 / 9007199254740992.0); }
    // Uniform in [lo, hi).
    double uniform(double lo, double hi) { return lo + (hi - lo) * nextDouble(); }

private:
    std::uint64_t state_;
};

}  // namespace wpiemu::sdk
