// Spinning multi-beam LiDAR.
//
// The sensor has C channels (lasers), each at a fixed elevation angle, and rotates at
// `rotationHz`, firing every channel at A evenly spaced azimuths per revolution. Firing j of a
// scan happens at time t_j = j / (A * rotationHz), so if the vehicle moves during the sweep, each
// column is fired from a slightly different pose (the "rolling shutter" distortion real raw scans
// have). Points are reported in the sensor frame at their own firing time, like raw sensor output.
//
// For each beam:
//   1. Ray-trace the scene (BVH) for the nearest surface at range r.
//   2. Received power (normalised): P = rho * cos(theta) * exp(-2*alpha*r) / r^2
//      (Lambertian target with reflectivity rho, incidence angle theta, fog extinction alpha).
//      The target is detected if P >= minPower.
//   3. In fog, a scattering event may happen first (see weather.hpp) and produce a fog return.
//   4. Optional random dropout, then Gaussian range noise with standard deviation sigma.
#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "lidarsim/scene.hpp"
#include "lidarsim/weather.hpp"

namespace lidarsim {

struct LidarConfig {
    std::vector<double> elevationDeg;  // one entry per channel
    int azimuthSteps = 2048;           // firings per channel per revolution
    double rotationHz = 10;
    double minRange = 0.5, maxRange = 120;  // metres
    double rangeNoiseSigma = 0.02;          // metres, 1 sigma
    double dropoutProbability = 0.0;
    // Detection threshold on P. A rho = 1 target facing the sensor in clear air is detectable
    // out to 1 / sqrt(minPower) metres (150 m by default).
    double minPower = 1.0 / (150.0 * 150.0);

    int channels() const { return static_cast<int>(elevationDeg.size()); }
    long raysPerScan() const { return static_cast<long>(channels()) * azimuthSteps; }

    static LidarConfig vlp16();  // 16 channels, -15..+15 deg, 1800 azimuths
    static LidarConfig hdl64();  // 64 channels, -24.9..+2 deg, 2048 azimuths
};

// Constant world-frame velocity and yaw rate during a scan.
struct Motion {
    Vec3 velocity;
    double yawRate = 0;  // rad/s
};

struct ScanOptions {
    Pose pose;  // sensor pose at the start of the scan (t = 0)
    Motion motion;
    FogConfig fog;
    uint64_t seed = 1;
    int frame = 0;
    int threads = 1;
    bool noise = true;  // false disables range noise and dropout (for analytic validation)
};

struct LidarPoint {
    float x, y, z;      // sensor frame at firing time, metres
    float intensity;    // rho * cos(theta) * two-way transmittance, in [0, 1]
    double range;       // measured (noisy) range; double so validation measures geometry, not float rounding
    double trueRange;   // exact ray-traced range: ground truth for validation
    float time;         // firing time within the scan, seconds
    uint16_t channel;
    uint16_t azimuth;
    int32_t object;     // scene object index, -1 for fog
    Label label;
};

class Lidar {
public:
    explicit Lidar(LidarConfig config) : config_(std::move(config)) {}

    const LidarConfig& config() const { return config_; }

    // Full revolution. Output order is fixed (azimuth-major, then channel), independent of threads.
    std::vector<LidarPoint> scan(const Scene& scene, const ScanOptions& options) const;

    // One beam. Returns true and fills `out` if it produces a return. Public so tests can probe it.
    bool fire(const Scene& scene, const ScanOptions& options, int channel, int azimuth, LidarPoint& out) const;

    double firingTime(int azimuth) const { return azimuth / (config_.azimuthSteps * config_.rotationHz); }
    Vec3 beamDirection(int channel, int azimuth) const;     // unit vector, sensor frame
    Pose poseAt(const ScanOptions& options, double t) const;  // sensor pose at time t

private:
    LidarConfig config_;
};

}  // namespace lidarsim
