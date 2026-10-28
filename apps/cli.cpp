// Command-line driver: simulate one or more LiDAR frames and write them to disk.
//
//   lidarsim_cli [--scene street|room] [--preset hdl64|vlp16] [--azimuth N] [--frames K]
//                [--fog VISIBILITY_M] [--speed MPS] [--yaw-rate RADPS] [--seed S]
//                [--threads T] [--out DIR] [--no-noise]
//
// Writes DIR/frame_NNNNNN.bin (KITTI), .csv (all fields + ground truth) and .ply for each frame.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <string>
#include <thread>

#include "lidarsim/io.hpp"
#include "lidarsim/lidar.hpp"

using namespace lidarsim;

static int run(int argc, char** argv) {
    std::string sceneName = "street", preset = "hdl64", outDir = "out";
    int frames = 1, azimuth = 0;
    int threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
    double fog = 0, speed = 0, yawRate = 0;
    uint64_t seed = 1;
    bool noise = true;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", a.c_str()); std::exit(2); }
            return argv[++i];
        };
        if (a == "--scene") sceneName = next();
        else if (a == "--preset") preset = next();
        else if (a == "--azimuth") azimuth = std::atoi(next());
        else if (a == "--frames") frames = std::atoi(next());
        else if (a == "--fog") fog = std::atof(next());
        else if (a == "--speed") speed = std::atof(next());
        else if (a == "--yaw-rate") yawRate = std::atof(next());
        else if (a == "--seed") seed = std::strtoull(next(), nullptr, 10);
        else if (a == "--threads") threads = std::atoi(next());
        else if (a == "--out") outDir = next();
        else if (a == "--no-noise") noise = false;
        else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
    }

    LidarConfig cfg = preset == "vlp16" ? LidarConfig::vlp16() : LidarConfig::hdl64();
    if (azimuth > 0) cfg.azimuthSteps = azimuth;
    const Lidar lidar(cfg);
    const Scene scene = sceneName == "room" ? makeRoomScene(20, 14, 4, 0.6) : makeStreetScene(42);

    ScanOptions opt;
    opt.pose = sceneName == "room" ? Pose{{1.3, -0.7, 1.7}, 0.4} : Pose{{-40, 0, 1.8}, 0};
    opt.motion = {{speed, 0, 0}, yawRate};
    opt.fog.visibility = fog;
    opt.seed = seed;
    opt.threads = threads;
    opt.noise = noise;

    std::filesystem::create_directories(outDir);
    std::printf("scene=%s triangles=%zu preset=%s channels=%d azimuth=%d rays/scan=%ld threads=%d fog=%s\n",
                sceneName.c_str(), scene.triangles().size(), preset.c_str(), cfg.channels(), cfg.azimuthSteps,
                cfg.raysPerScan(), threads, fog > 0 ? (std::to_string(fog) + " m").c_str() : "off");

    for (int f = 0; f < frames; ++f) {
        opt.frame = f;
        const auto start = std::chrono::steady_clock::now();
        const auto points = lidar.scan(scene, opt);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

        char base[512];
        std::snprintf(base, sizeof base, "%s/frame_%06d", outDir.c_str(), f);
        writeKittiBin(std::string(base) + ".bin", points);
        writeCsv(std::string(base) + ".csv", points);
        writePly(std::string(base) + ".ply", points);
        std::printf("frame %d: %zu points in %.2f ms -> %s.{bin,csv,ply}\n", f, points.size(), ms, base);

        // The next frame starts where this revolution ended.
        opt.pose = lidar.poseAt(opt, 1.0 / cfg.rotationHz);
    }
    return 0;
}

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
