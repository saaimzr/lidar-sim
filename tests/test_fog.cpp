// Fog model checked against its own equations (see weather.hpp).
#include <fstream>

#include "lidarsim/lidar.hpp"
#include "test.hpp"

using namespace lidarsim;

namespace {

// One forward-looking beam: channel 0 at 0 deg elevation, azimuth 0 points along +x.
LidarConfig singleBeam() {
    LidarConfig c;
    c.elevationDeg = {0.0};
    c.azimuthSteps = 1;
    c.maxRange = 300;
    return c;
}

// A wall facing the sensor: front face on the plane x = 0, large enough to fill the beam.
Scene wallScene(double reflectivity) {
    Scene s;
    s.add({makeBox(0.2, 40, 40), Pose{{0.1, 0, -20}, 0}, reflectivity, Label::Wall});
    s.build();
    return s;
}

// Largest range r at which rho * exp(-2 alpha r) / r^2 >= minPower (perpendicular target),
// found by bisection on the monotone log form: ln(rho) - 2 alpha r - 2 ln(r) - ln(minPower).
double analyticMaxRange(double rho, double alpha, double minPower) {
    auto f = [&](double r) { return std::log(rho) - 2 * alpha * r - 2 * std::log(r) - std::log(minPower); };
    double lo = 0.01, hi = 1000;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        (f(mid) >= 0 ? lo : hi) = mid;
    }
    return lo;
}

}  // namespace

TEST(fog_detection_range_matches_link_budget) {
    // Move the sensor back from the wall in 1 cm steps and record the farthest distance at which
    // the wall is still detected; compare with the analytic solution of the link budget.
    constexpr double kRho = 0.8, kStep = 0.01;
    const Scene wall = wallScene(kRho);
    const Lidar lidar(singleBeam());
    std::ofstream csv(validationDir() + "/fog_range.csv");
    csv << "visibility_m,analytic_max_range_m,simulated_max_range_m\n";

    double maxAbs = 0, maxRel = 0;
    std::string tested;
    for (double visibility : {20.0, 50.0, 100.0, 200.0, 500.0, 0.0}) {  // 0 = clear air
        ScanOptions opt;
        opt.noise = false;
        opt.fog.visibility = visibility;
        opt.fog.backscatterStrength = 0;  // isolate attenuation: no fog returns in this test
        const double expected = analyticMaxRange(kRho, opt.fog.extinction(), lidar.config().minPower);

        double lastDetected = 0;
        for (double d = 1.0; d < 250; d += kStep) {
            opt.pose = {{-d, 0, 0}, 0};
            LidarPoint p;
            if (lidar.fire(wall, opt, 0, 0, p) && p.label == Label::Wall) lastDetected = d;
        }
        const double err = std::fabs(lastDetected - expected);
        maxAbs = std::max(maxAbs, err);
        maxRel = std::max(maxRel, err / expected);
        csv << visibility << "," << expected << "," << lastDetected << "\n";
        tested += (tested.empty() ? "" : ",") + (visibility > 0 ? std::to_string(static_cast<int>(visibility)) : std::string("clear"));
        CHECK(err <= kStep + 1e-9);
    }
    metric("fog_range.visibilities_m", tested);
    metric("fog_range.sweep_step_m", kStep);
    metric("fog_range.max_abs_discrepancy_m", maxAbs);
    metric("fog_range.max_rel_discrepancy_percent", 100 * maxRel);
}

TEST(fog_backscatter_probability_matches_exponential_law) {
    // With a wall at 20 m and no power threshold on fog returns, a beam returns from fog exactly
    // when the sampled scatter distance d falls in [minRange, 20): probability
    // exp(-alpha * minRange) - exp(-alpha * 20). Compare the observed frequency over 200k beams.
    const Scene wall = wallScene(0.8);
    const LidarConfig cfg = singleBeam();
    const Lidar lidar(cfg);
    constexpr double kWall = 20;
    constexpr long long kTrials = 200000;
    for (double visibility : {50.0, 200.0}) {
        ScanOptions opt;
        opt.noise = false;
        opt.fog.visibility = visibility;
        opt.fog.backscatterStrength = 1e12;  // every scattering event is detectable
        opt.pose = {{-kWall, 0, 0}, 0};
        const double alpha = opt.fog.extinction();
        const double expected = std::exp(-alpha * cfg.minRange) - std::exp(-alpha * kWall);

        // Same beam every time; each trial gets an independent random stream through its seed.
        long long fogReturns = 0;
        for (long long k = 0; k < kTrials; ++k) {
            opt.seed = static_cast<uint64_t>(k) + 1;
            LidarPoint p;
            if (lidar.fire(wall, opt, 0, 0, p) && p.label == Label::Fog) ++fogReturns;
        }
        const long long trials = kTrials;
        const double observed = static_cast<double>(fogReturns) / trials;
        const double se = std::sqrt(expected * (1 - expected) / trials);
        const std::string key = "fog_backscatter.V" + std::to_string(static_cast<int>(visibility)) + "m.";
        metric(key + "trials", trials);
        metric(key + "expected_fraction", expected);
        metric(key + "observed_fraction", observed);
        metric(key + "difference_percentage_points", 100 * (observed - expected));
        metric(key + "z_score", (observed - expected) / se);
        CHECK(std::fabs(observed - expected) < 4 * se);
    }
}
