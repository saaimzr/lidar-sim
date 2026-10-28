#include "lidarsim/lidar.hpp"

#include <algorithm>
#include <thread>

#include "lidarsim/rng.hpp"

namespace lidarsim {

LidarConfig LidarConfig::vlp16() {
    LidarConfig c;
    for (int i = 0; i < 16; ++i) c.elevationDeg.push_back(-15.0 + 2.0 * i);
    c.azimuthSteps = 1800;
    c.maxRange = 100;
    return c;
}

LidarConfig LidarConfig::hdl64() {
    LidarConfig c;
    for (int i = 0; i < 64; ++i) c.elevationDeg.push_back(-24.9 + (2.0 + 24.9) * i / 63.0);
    c.azimuthSteps = 2048;
    c.maxRange = 120;
    return c;
}

Vec3 Lidar::beamDirection(int channel, int azimuth) const {
    const double el = deg2rad(config_.elevationDeg[channel]);
    const double az = 2.0 * kPi * azimuth / config_.azimuthSteps;  // counter-clockwise from +x
    return {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
}

Pose Lidar::poseAt(const ScanOptions& options, double t) const {
    Pose p = options.pose;
    p.position += options.motion.velocity * t;
    p.yaw += options.motion.yawRate * t;
    return p;
}

bool Lidar::fire(const Scene& scene, const ScanOptions& options, int channel, int azimuth, LidarPoint& out) const {
    const double time = firingTime(azimuth);
    const Pose pose = poseAt(options, time);
    const Vec3 dirLocal = beamDirection(channel, azimuth);
    const Ray ray{pose.position, pose.rotate(dirLocal)};

    // Draw every random number up front, in a fixed order, so the sequence never depends on
    // which branches are taken below.
    Rng rng(options.seed, static_cast<uint64_t>(options.frame), static_cast<uint64_t>(channel),
            static_cast<uint64_t>(azimuth));
    const double uScatter = rng.uniform();
    const double uDropout = rng.uniform();
    const double gaussian = rng.normal();

    const double alpha = options.fog.extinction();
    const Hit hit = scene.intersect(ray, 1e-9, config_.maxRange);

    double range = 0, intensity = 0;
    int object = -1;
    Label label = Label::None;
    bool returned = false;

    // Hard target.
    if (hit.valid() && hit.t >= config_.minRange) {
        const Object& obj = scene.objectOf(hit.triangle);
        const double cosTheta = std::fabs(dot(scene.normalOf(hit.triangle), ray.dir));
        const double echo = obj.reflectivity * cosTheta * transmittance(alpha, hit.t);
        if (echo / (hit.t * hit.t) >= config_.minPower) {
            range = hit.t;
            intensity = echo;
            object = scene.objectIndexOf(hit.triangle);
            label = obj.label;
            returned = true;
        }
    }

    // Fog backscatter: first-return sensor, so a scattering event in front of the target wins.
    if (alpha > 0) {
        const double d = sampleScatterDistance(alpha, uScatter);
        const double limit = hit.valid() ? hit.t : config_.maxRange;
        if (d >= config_.minRange && d < limit) {
            const double echo = options.fog.backscatterStrength * transmittance(alpha, d);
            if (echo / (d * d) >= config_.minPower) {
                range = d;
                intensity = echo;
                object = -1;
                label = Label::Fog;
                returned = true;
            }
        }
    }

    if (!returned) return false;
    if (options.noise && uDropout < config_.dropoutProbability) return false;

    const double measured = options.noise ? range + config_.rangeNoiseSigma * gaussian : range;
    const Vec3 p = dirLocal * measured;
    out.x = static_cast<float>(p.x);
    out.y = static_cast<float>(p.y);
    out.z = static_cast<float>(p.z);
    out.intensity = static_cast<float>(intensity);
    out.range = measured;
    out.trueRange = range;
    out.time = static_cast<float>(time);
    out.channel = static_cast<uint16_t>(channel);
    out.azimuth = static_cast<uint16_t>(azimuth);
    out.object = object;
    out.label = label;
    return true;
}

std::vector<LidarPoint> Lidar::scan(const Scene& scene, const ScanOptions& options) const {
    const int C = config_.channels(), A = config_.azimuthSteps;
    std::vector<LidarPoint> slots(static_cast<size_t>(C) * A);
    std::vector<uint8_t> hasReturn(slots.size(), 0);

    // Each worker owns a contiguous block of azimuth columns and writes only its own slots,
    // so no locking is needed. Per-beam RNG makes the result independent of the split.
    auto work = [&](int azBegin, int azEnd) {
        for (int a = azBegin; a < azEnd; ++a)
            for (int c = 0; c < C; ++c) {
                const size_t i = static_cast<size_t>(a) * C + c;
                hasReturn[i] = fire(scene, options, c, a, slots[i]);
            }
    };
    const int threads = std::max(1, std::min(options.threads, A));
    if (threads == 1) {
        work(0, A);
    } else {
        std::vector<std::thread> pool;
        for (int t = 0; t < threads; ++t) pool.emplace_back(work, A * t / threads, A * (t + 1) / threads);
        for (auto& th : pool) th.join();
    }

    std::vector<LidarPoint> points;
    points.reserve(slots.size());
    for (size_t i = 0; i < slots.size(); ++i)
        if (hasReturn[i]) points.push_back(slots[i]);
    return points;
}

}  // namespace lidarsim
