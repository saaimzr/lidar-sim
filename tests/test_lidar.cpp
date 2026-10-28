// Sensor-level validation: full scans compared against closed-form geometry and statistics.
#include <algorithm>
#include <fstream>
#include <set>

#include "lidarsim/io.hpp"
#include "lidarsim/lidar.hpp"
#include "test.hpp"

using namespace lidarsim;

namespace {

constexpr double kRoomX = 20, kRoomY = 14, kRoomZ = 4;

// Distance from a point inside an axis-aligned box to its wall along `d`, written independently
// of the triangle code: for each axis, the plane the ray is heading towards, nearest one wins.
double analyticBoxExit(const Vec3& o, const Vec3& d) {
    const Vec3 lo{-kRoomX / 2, -kRoomY / 2, 0}, hi{kRoomX / 2, kRoomY / 2, kRoomZ};
    double t = kInf;
    for (int axis = 0; axis < 3; ++axis) {
        if (d[axis] > 0) t = std::min(t, (hi[axis] - o[axis]) / d[axis]);
        if (d[axis] < 0) t = std::min(t, (lo[axis] - o[axis]) / d[axis]);
    }
    return t;
}

// Beam direction in the world, rebuilt from first principles (not via Lidar::beamDirection).
Vec3 worldBeam(double elevationDeg, int azimuth, int steps, double yaw) {
    const double el = elevationDeg * kPi / 180, az = 2 * kPi * azimuth / steps + yaw;
    return {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
}

LidarConfig exactConfig() {  // no noise, no range or power cut-off
    LidarConfig c = LidarConfig::hdl64();
    c.minPower = 1e-12;
    c.maxRange = 10000;
    return c;
}

uint64_t hashScan(const std::vector<LidarPoint>& pts) {
    uint64_t h = 1469598103934665603ull;
    for (const auto& p : pts) {  // hash fields one by one so struct padding never matters
        h = fnv1a(&p.x, sizeof p.x, h); h = fnv1a(&p.y, sizeof p.y, h); h = fnv1a(&p.z, sizeof p.z, h);
        h = fnv1a(&p.intensity, sizeof p.intensity, h); h = fnv1a(&p.range, sizeof p.range, h);
        h = fnv1a(&p.channel, sizeof p.channel, h); h = fnv1a(&p.azimuth, sizeof p.azimuth, h);
        h = fnv1a(&p.object, sizeof p.object, h); h = fnv1a(&p.label, sizeof p.label, h);
    }
    return h;
}

}  // namespace

TEST(room_scan_matches_analytic_ranges) {
    const Scene room = makeRoomScene(kRoomX, kRoomY, kRoomZ, 0.6);
    const Lidar lidar(exactConfig());
    ScanOptions opt;
    opt.pose = {{1.3, -0.7, 1.7}, 0.4};
    opt.noise = false;
    opt.threads = 8;
    const auto pts = lidar.scan(room, opt);

    // Every beam must hit a wall: one return per (channel, azimuth).
    CHECK(static_cast<long>(pts.size()) == lidar.config().raysPerScan());
    double maxErr = 0, sumErr = 0;
    std::vector<double> errs;
    for (const auto& p : pts) {
        const Vec3 d = worldBeam(lidar.config().elevationDeg[p.channel], p.azimuth, lidar.config().azimuthSteps, opt.pose.yaw);
        const double err = std::fabs(p.range - analyticBoxExit(opt.pose.position, d));
        maxErr = std::max(maxErr, err);
        sumErr += err;
        errs.push_back(err);
    }
    std::nth_element(errs.begin(), errs.begin() + errs.size() / 2, errs.end());
    metric("analytic_room.rays_tested", static_cast<long long>(lidar.config().raysPerScan()));
    metric("analytic_room.returns", static_cast<long long>(pts.size()));
    metric("analytic_room.max_abs_range_error_m", maxErr);
    metric("analytic_room.mean_abs_range_error_m", sumErr / pts.size());
    metric("analytic_room.median_abs_range_error_m", errs[errs.size() / 2]);
    CHECK(maxErr < 1e-9);
}

TEST(ground_plane_ranges_and_intensity) {
    // Sensor h metres above an (effectively infinite) flat ground. A beam at elevation el < 0
    // reaches the ground at range h / sin(-el), and hits it at incidence cos(theta) = sin(-el),
    // so intensity = rho * sin(-el). Independent of azimuth and of the sensor's yaw.
    Scene ground;
    ground.add({makePlane(4000), Pose{}, 0.35, Label::Ground});
    ground.build();
    const Lidar lidar(exactConfig());
    ScanOptions opt;
    opt.pose = {{0, 0, 1.8}, 0.3};
    opt.noise = false;
    opt.threads = 8;
    const auto pts = lidar.scan(ground, opt);

    double maxRangeErr = 0, maxIntensityErr = 0;
    long long expected = 0;
    for (double el : lidar.config().elevationDeg) expected += el < 0 ? lidar.config().azimuthSteps : 0;
    for (const auto& p : pts) {
        const double s = std::sin(-lidar.config().elevationDeg[p.channel] * kPi / 180);
        maxRangeErr = std::max(maxRangeErr, std::fabs(p.range - 1.8 / s));
        maxIntensityErr = std::max(maxIntensityErr, std::fabs(p.intensity - 0.35 * s));
    }
    metric("ground_plane.returns", static_cast<long long>(pts.size()));
    metric("ground_plane.max_abs_range_error_m", maxRangeErr);
    metric("ground_plane.max_abs_intensity_error", maxIntensityErr);
    CHECK(static_cast<long long>(pts.size()) == expected);
    CHECK(maxRangeErr < 1e-6);  // ranges reach ~740 m here
    CHECK(maxIntensityErr < 1e-6);
}

TEST(moving_sensor_rolling_shutter_matches_analytic) {
    // Each column is fired from the pose at its own time t = a / (A * f).
    const Scene room = makeRoomScene(kRoomX, kRoomY, kRoomZ, 0.6);
    const Lidar lidar(exactConfig());
    ScanOptions opt;
    opt.pose = {{-3, 0.5, 1.7}, 0.2};
    opt.motion = {{8, 1.5, 0}, 0.6};  // 8 m/s forward, 1.5 m/s sideways, 0.6 rad/s yaw
    opt.noise = false;
    opt.threads = 8;
    const auto moving = lidar.scan(room, opt);
    ScanOptions still = opt;
    still.motion = {};
    const auto fixed = lidar.scan(room, still);
    CHECK(moving.size() == fixed.size());

    const auto& cfg = lidar.config();
    double maxErr = 0, maxSkew = 0;
    for (size_t i = 0; i < moving.size(); ++i) {
        const auto& p = moving[i];
        const double t = p.azimuth / (cfg.azimuthSteps * cfg.rotationHz);
        const Vec3 origin = opt.pose.position + opt.motion.velocity * t;
        const Vec3 d = worldBeam(cfg.elevationDeg[p.channel], p.azimuth, cfg.azimuthSteps, opt.pose.yaw + opt.motion.yawRate * t);
        maxErr = std::max(maxErr, std::fabs(p.range - analyticBoxExit(origin, d)));
        maxSkew = std::max(maxSkew, std::fabs(p.range - fixed[i].range));
    }
    metric("rolling_shutter.max_abs_range_error_m", maxErr);
    metric("rolling_shutter.max_shift_vs_static_scan_m", maxSkew);
    CHECK(maxErr < 1e-9);
    CHECK(maxSkew > 0.3);  // the motion really does distort the scan
}

TEST(range_noise_statistics) {
    // Gaussian range noise: over n returns the sample mean must be within 4 standard errors of 0
    // and the sample standard deviation within 1% of the configured sigma.
    const Scene room = makeRoomScene(kRoomX, kRoomY, kRoomZ, 0.6);
    LidarConfig cfg = exactConfig();
    cfg.rangeNoiseSigma = 0.02;
    const Lidar lidar(cfg);
    ScanOptions opt;
    opt.pose = {{1.3, -0.7, 1.7}, 0.4};
    opt.threads = 8;

    double sum = 0, sumSq = 0;
    long long n = 0;
    std::ofstream sample(validationDir() + "/noise_errors.csv");
    sample << "error_m\n";
    for (int frame = 0; n < 200000; ++frame) {
        opt.frame = frame;
        for (const auto& p : lidar.scan(room, opt)) {
            const double e = p.range - p.trueRange;
            sum += e;
            sumSq += e * e;
            if (n < 50000) sample << e << "\n";
            ++n;
        }
    }
    const double mean = sum / n;
    const double sd = std::sqrt(sumSq / n - mean * mean);
    metric("noise.configured_sigma_m", cfg.rangeNoiseSigma);
    metric("noise.samples", n);
    metric("noise.measured_mean_m", mean);
    metric("noise.measured_std_m", sd);
    metric("noise.std_relative_error_percent", 100 * std::fabs(sd / cfg.rangeNoiseSigma - 1));
    CHECK(std::fabs(mean) < 4 * cfg.rangeNoiseSigma / std::sqrt(static_cast<double>(n)));
    CHECK(std::fabs(sd / cfg.rangeNoiseSigma - 1) < 0.01);
}

TEST(dropout_rate_matches_configuration) {
    const Scene room = makeRoomScene(kRoomX, kRoomY, kRoomZ, 0.6);
    LidarConfig cfg = exactConfig();
    cfg.dropoutProbability = 0.1;
    const Lidar lidar(cfg);
    ScanOptions opt;
    opt.pose = {{1.3, -0.7, 1.7}, 0.4};
    opt.threads = 8;
    const double n = static_cast<double>(cfg.raysPerScan());
    const double dropped = 1.0 - lidar.scan(room, opt).size() / n;
    metric("dropout.configured", cfg.dropoutProbability);
    metric("dropout.measured", dropped);
    CHECK(std::fabs(dropped - 0.1) < 4 * std::sqrt(0.1 * 0.9 / n));
}

TEST(scan_is_identical_for_any_thread_count) {
    // Noise, dropout, fog and motion all on, so every random draw is exercised.
    const Scene street = makeStreetScene(42);
    LidarConfig cfg = LidarConfig::hdl64();
    cfg.dropoutProbability = 0.02;
    const Lidar lidar(cfg);
    ScanOptions opt;
    opt.pose = {{0, 0, 1.8}, 0};
    opt.motion = {{10, 0, 0}, 0.1};
    opt.fog.visibility = 80;
    opt.seed = 7;

    opt.threads = 1;
    const uint64_t reference = hashScan(lidar.scan(street, opt));
    std::string tried = "1";
    for (int threads : {2, 3, 4, 8, 12, 16}) {
        opt.threads = threads;
        CHECK(hashScan(lidar.scan(street, opt)) == reference);
        tried += "," + std::to_string(threads);
    }
    opt.threads = 8;
    opt.seed = 8;
    CHECK(hashScan(lidar.scan(street, opt)) != reference);  // a different seed must change the noise
    char hex[17];
    std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(reference));
    metric("determinism.thread_counts_compared", tried);
    metric("determinism.identical", std::string("yes"));
    metric("determinism.fnv1a64_hash", std::string(hex));
}

TEST(labels_and_ground_truth_are_consistent) {
    const Scene street = makeStreetScene(42);
    const Lidar lidar(LidarConfig::hdl64());
    ScanOptions opt;
    opt.pose = {{0, 0, 1.8}, 0};
    opt.noise = false;
    opt.threads = 8;
    const auto pts = lidar.scan(street, opt);
    std::set<int> labels;
    for (const auto& p : pts) {
        CHECK(p.object >= 0 && p.label == street.object(p.object).label);
        CHECK_NEAR(std::sqrt(double(p.x) * p.x + double(p.y) * p.y + double(p.z) * p.z), p.trueRange, 1e-4);
        labels.insert(static_cast<int>(p.label));
    }
    metric("street.returns_noise_off", static_cast<long long>(pts.size()));
    metric("street.semantic_classes_seen", static_cast<long long>(labels.size()));
    CHECK(labels.size() >= 5);
}

TEST(kitti_bin_round_trip) {
    const Scene street = makeStreetScene(3);
    const Lidar lidar(LidarConfig::vlp16());
    ScanOptions opt;
    opt.threads = 4;
    opt.pose = {{0, 0, 1.8}, 0};
    const auto pts = lidar.scan(street, opt);
    const std::string path = validationDir() + "/roundtrip.bin";
    writeKittiBin(path, pts);
    const auto back = readKittiBin(path);
    CHECK(back.size() == pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
        CHECK(back[i].x == pts[i].x && back[i].y == pts[i].y && back[i].z == pts[i].z &&
              back[i].intensity == pts[i].intensity);
    std::remove(path.c_str());
}
