// Fog model.
//
// Fog is treated as a uniform scattering medium with extinction coefficient alpha (1/m):
//  * Attenuation (Beer–Lambert): light that travels to a target at range r and back keeps
//    a fraction exp(-2 * alpha * r) of its power.
//  * Backscatter: the distance a photon travels before it first scatters is exponentially
//    distributed with rate alpha, so P(scatter before range R) = 1 - exp(-alpha * R). If that
//    happens before the beam reaches a target, the sensor can register a spurious "fog" return.
//  * Visibility V (meteorological optical range) relates to alpha by Koschmieder's law:
//    alpha = ln(20) / V, i.e. contrast drops to 5% after V metres.
#pragma once

#include <cmath>

namespace lidarsim {

struct FogConfig {
    double visibility = 0;             // metres; 0 means clear air (no fog)
    double backscatterStrength = 0.05;  // effective reflectivity of a fog return

    bool enabled() const { return visibility > 0; }
    double extinction() const { return enabled() ? std::log(20.0) / visibility : 0.0; }
};

// Two-way transmittance to range r.
inline double transmittance(double alpha, double r) { return std::exp(-2.0 * alpha * r); }

// Inverse-CDF sample of the free path before scattering, given u uniform in [0, 1).
inline double sampleScatterDistance(double alpha, double u) { return -std::log(1.0 - u) / alpha; }

}  // namespace lidarsim
