// Performance benchmark.
//
// Traces the 131,072 rays of one 64-beam scan through the street scene three ways:
//   brute force (1 thread), BVH (1 thread), BVH (N threads)
// times complete sensor scans, and reports BVH work per ray (nodes visited, triangles tested)
// for the street with its ground split into 2 m cells versus as two giant triangles.
// Every timing is the median of several runs.
//
//   lidarsim_bench [--threads N] [--runs R] [--detail D]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "lidarsim/lidar.hpp"

using namespace lidarsim;
using Clock = std::chrono::steady_clock;

namespace {

double medianSeconds(int runs, const std::function<void()>& fn) {
    std::vector<double> t;
    for (int r = 0; r < runs; ++r) {
        const auto start = Clock::now();
        fn();
        t.push_back(std::chrono::duration<double>(Clock::now() - start).count());
    }
    std::sort(t.begin(), t.end());
    return t[t.size() / 2];
}

// Trace rays[i] for i in [0, n) with the given stride, split across `threads`. The checksum keeps
// the compiler from optimising the work away and confirms both methods agree.
double traceRays(const Scene& scene, const std::vector<Ray>& rays, int stride, int threads, bool useBvh) {
    std::vector<double> partial(threads, 0.0);
    auto work = [&](int t) {
        double sum = 0;
        for (size_t i = static_cast<size_t>(t) * stride; i < rays.size(); i += static_cast<size_t>(threads) * stride) {
            const Hit h = useBvh ? scene.intersect(rays[i], 1e-9, 120)
                                 : intersectBruteForce(scene.triangles(), rays[i], 1e-9, 120);
            if (h.valid()) sum += h.t;
        }
        partial[t] = sum;
    };
    if (threads == 1) {
        work(0);
    } else {
        std::vector<std::thread> pool;
        for (int t = 0; t < threads; ++t) pool.emplace_back(work, t);
        for (auto& th : pool) th.join();
    }
    double total = 0;
    for (double p : partial) total += p;
    return total;
}

}  // namespace

int main(int argc, char** argv) {
    int threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
    int runs = 7, detail = 1;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string a = argv[i];
        if (a == "--threads") threads = std::atoi(argv[i + 1]);
        else if (a == "--runs") runs = std::atoi(argv[i + 1]);
        else if (a == "--detail") detail = std::atoi(argv[i + 1]);
    }

    const auto buildStart = Clock::now();
    const Scene scene = makeStreetScene(42, detail);
    const double sceneMs = std::chrono::duration<double, std::milli>(Clock::now() - buildStart).count();

    const LidarConfig cfg = LidarConfig::hdl64();
    const Lidar lidar(cfg);
    ScanOptions opt;
    opt.pose = {{0, 0, 1.8}, 0};

    // The exact rays of one scan (static sensor), in firing order.
    std::vector<Ray> rays;
    for (int a = 0; a < cfg.azimuthSteps; ++a)
        for (int c = 0; c < cfg.channels(); ++c)
            rays.push_back({opt.pose.position, opt.pose.rotate(lidar.beamDirection(c, a))});

    std::printf("hardware threads: %u\n", std::thread::hardware_concurrency());
    std::printf("scene: %zu triangles, BVH %d nodes, depth %d (scene + BVH build %.1f ms)\n", scene.triangles().size(),
                scene.bvh().nodeCount(), scene.bvh().depth(), sceneMs);
    std::printf("rays per scan: %zu (%d channels x %d azimuths), median of %d runs\n\n", rays.size(), cfg.channels(),
                cfg.azimuthSteps, runs);

    // BVH work per ray: how much of the tree a typical ray touches.
    auto workPerRay = [&](const Scene& s, const char* name) {
        TraversalStats st;
        for (const auto& r : rays) s.intersect(r, 1e-9, 120, &st);
        const double t = medianSeconds(runs, [&] { traceRays(s, rays, 1, 1, true); });
        std::printf("%-30s %7zu tris %7.1f nodes/ray %7.1f tri tests/ray %7.3f M rays/s (1 thread)\n", name,
                    s.triangles().size(), double(st.nodesVisited) / rays.size(), double(st.trianglesTested) / rays.size(),
                    rays.size() / t / 1e6);
    };
    std::printf("BVH work per ray\n");
    workPerRay(scene, "ground split into 2 m cells");
    workPerRay(makeStreetScene(42, detail, 1), "ground as 2 giant triangles");
    std::printf("\n");

    // Fast measurements first; the long brute-force run goes last so the heat it builds up
    // (laptops throttle under sustained load) doesn't slow the other measurements.
    const double tBvh1 = medianSeconds(runs, [&] { traceRays(scene, rays, 1, 1, true); });
    const double tBvhN = medianSeconds(runs, [&] { traceRays(scene, rays, 1, threads, true); });

    // Complete sensor scans: ray tracing + sensor model + output assembly.
    size_t points = 0;
    opt.threads = 1;
    const double scan1 = medianSeconds(runs, [&] { points = lidar.scan(scene, opt).size(); });
    opt.threads = threads;
    const double scanN = medianSeconds(runs, [&] { lidar.scan(scene, opt); });
    opt.fog.visibility = 50;
    const double scanFog = medianSeconds(runs, [&] { lidar.scan(scene, opt); });

    // Brute force is far slower, so it traces every 16th ray; rays/s stays comparable.
    constexpr int kBruteStride = 16;
    const size_t bruteRays = (rays.size() + kBruteStride - 1) / kBruteStride;
    double sumBrute = 0;
    const double tBrute = medianSeconds(3, [&] { sumBrute = traceRays(scene, rays, kBruteStride, 1, false); });
    const double checkBvh = traceRays(scene, rays, kBruteStride, 1, true);

    const double rBrute = bruteRays / tBrute, rBvh1 = rays.size() / tBvh1, rBvhN = rays.size() / tBvhN;
    std::printf("%-26s %10s %12s %14s\n", "ray tracing", "rays", "median ms", "M rays/s");
    std::printf("%-26s %10zu %12.2f %14.3f\n", "brute force, 1 thread", bruteRays, 1e3 * tBrute, rBrute / 1e6);
    std::printf("%-26s %10zu %12.2f %14.3f\n", "BVH, 1 thread", rays.size(), 1e3 * tBvh1, rBvh1 / 1e6);
    char label[64];
    std::snprintf(label, sizeof label, "BVH, %d threads", threads);
    std::printf("%-26s %10zu %12.2f %14.3f\n", label, rays.size(), 1e3 * tBvhN, rBvhN / 1e6);
    std::printf("speedup: BVH vs brute %.0fx, threads vs 1T %.2fx, total %.0fx\n", rBvh1 / rBrute, rBvhN / rBvh1, rBvhN / rBrute);
    std::printf("checksum (strided subset) brute %.6f vs BVH %.6f -> %s\n\n", sumBrute, checkBvh,
                std::fabs(sumBrute - checkBvh) < 1e-6 ? "match" : "MISMATCH");

    std::printf("full 64-beam scan (%zu rays, %zu returns):\n", rays.size(), points);
    std::printf("  1 thread:              %8.2f ms/scan  (%6.1f scans/s)\n", 1e3 * scan1, 1 / scan1);
    std::printf("  %2d threads:            %8.2f ms/scan  (%6.1f scans/s)\n", threads, 1e3 * scanN, 1 / scanN);
    std::printf("  %2d threads, fog 50 m:  %8.2f ms/scan  (%6.1f scans/s)\n", threads, 1e3 * scanFog, 1 / scanFog);

    std::ofstream csv("validation/bench.csv");
    csv << "method,rays,median_s,rays_per_s\n"
        << "brute_1t," << bruteRays << "," << tBrute << "," << rBrute << "\n"
        << "bvh_1t," << rays.size() << "," << tBvh1 << "," << rBvh1 << "\n"
        << "bvh_" << threads << "t," << rays.size() << "," << tBvhN << "," << rBvhN << "\n";
    return 0;
}
