// The BVH must return exactly what testing every triangle returns.
#include "lidarsim/bvh.hpp"
#include "lidarsim/rng.hpp"
#include "lidarsim/scene.hpp"
#include "test.hpp"

using namespace lidarsim;

namespace {

// Random "triangle soup": n small triangles scattered through a cube of side `extent`.
std::vector<Triangle> randomSoup(uint64_t seed, int n, double extent) {
    Rng rng(seed, 7, 0, 0);
    std::vector<Triangle> tris;
    for (int i = 0; i < n; ++i) {
        const Vec3 c{rng.uniform(0, extent), rng.uniform(0, extent), rng.uniform(0, extent)};
        auto jitter = [&] { return c + Vec3{rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1)}; };
        const Vec3 a = jitter();
        const Vec3 b = jitter();
        const Vec3 d = jitter();
        tris.push_back({a, b, d});
    }
    return tris;
}

// Uniformly distributed unit vector (normalised Gaussian triple).
Vec3 randomDirection(Rng& rng) {
    const double x = rng.normal();
    const double y = rng.normal();
    const double z = rng.normal();
    return normalize({x, y, z});
}

}  // namespace

TEST(bvh_matches_brute_force_on_randomized_rays) {
    constexpr int kScenes = 10, kRaysPerScene = 10000;
    long long rays = 0, hits = 0, hitMissMismatch = 0, distanceMismatch = 0, idMismatch = 0, ties = 0;
    double maxDt = 0;
    int totalTriangles = 0;

    for (int s = 0; s < kScenes; ++s) {
        // 8 random soups of increasing size plus 2 procedural street scenes.
        std::vector<Triangle> tris;
        Scene street;
        if (s < 8) {
            tris = randomSoup(100 + s, 500 + 700 * s, 40);
        } else {
            street = makeStreetScene(200 + s);
            tris = street.triangles();
        }
        totalTriangles += static_cast<int>(tris.size());
        Bvh bvh;
        bvh.build(tris);

        AABB bounds;
        for (const auto& t : tris) { bounds.expand(t.a); bounds.expand(t.b); bounds.expand(t.c); }
        Rng rng(999, s, 0, 0);
        for (int r = 0; r < kRaysPerScene; ++r) {
            const Vec3 o{rng.uniform(bounds.lo.x, bounds.hi.x), rng.uniform(bounds.lo.y, bounds.hi.y),
                         rng.uniform(bounds.lo.z + 0.01, bounds.hi.z)};
            // Odd rays aim at a random point on a random triangle so most comparisons are real hits;
            // even rays go in a uniformly random direction (and often miss everything).
            Vec3 dir = randomDirection(rng);
            if (r % 2 == 1) {
                const Triangle& target = tris[static_cast<size_t>(rng.uniform() * tris.size()) % tris.size()];
                double u = rng.uniform(), v = rng.uniform();
                if (u + v > 1) { u = 1 - u; v = 1 - v; }
                dir = normalize(target.a + (target.b - target.a) * u + (target.c - target.a) * v - o);
            }
            const Ray ray{o, dir};
            const Hit a = bvh.intersect(ray, 1e-9, kInf);
            const Hit b = intersectBruteForce(tris, ray, 1e-9, kInf);
            ++rays;
            if (a.valid() != b.valid()) { ++hitMissMismatch; continue; }
            if (!a.valid()) continue;
            ++hits;
            const double dt = std::fabs(a.t - b.t);
            maxDt = std::max(maxDt, dt);
            if (dt > 1e-9) ++distanceMismatch;
            // A ray through an edge shared by two triangles can hit both at the same distance (up to
            // rounding), and either answer is correct. Prove that is what happened: the other
            // method's triangle must also be hit at that distance. Anything else is a real mismatch.
            if (a.triangle != b.triangle) {
                double ta = 0, tb = 0;
                const bool bothHit = intersectTriangle(ray, tris[a.triangle], 1e-9, kInf, ta) &&
                                     intersectTriangle(ray, tris[b.triangle], 1e-9, kInf, tb);
                if (bothHit && std::fabs(ta - tb) < 1e-9) ++ties;
                else ++idMismatch;
            }
        }
    }
    metric("bvh.random_scenes", static_cast<long long>(kScenes));
    metric("bvh.total_triangles", static_cast<long long>(totalTriangles));
    metric("bvh.total_rays", rays);
    metric("bvh.rays_that_hit", hits);
    metric("bvh.hit_miss_mismatches", hitMissMismatch);
    metric("bvh.distance_mismatches(>1e-9m)", distanceMismatch);
    metric("bvh.triangle_id_mismatches", idMismatch);
    metric("bvh.verified_equal_distance_ties", ties);
    metric("bvh.max_abs_t_difference_m", maxDt);
    CHECK(hitMissMismatch == 0);
    CHECK(distanceMismatch == 0);
    CHECK(idMismatch == 0);
}

TEST(bvh_handles_empty_and_single_triangle) {
    Bvh empty;
    std::vector<Triangle> none;
    empty.build(none);
    CHECK(!empty.intersect({{0, 0, 0}, {1, 0, 0}}, 0, kInf).valid());

    std::vector<Triangle> one{{{5, -1, -1}, {5, 1, -1}, {5, 0, 1}}};
    Bvh bvh;
    bvh.build(one);
    const Hit h = bvh.intersect({{0, 0, 0}, {1, 0, 0}}, 0, kInf);
    CHECK(h.valid() && h.triangle == 0);
    CHECK_NEAR(h.t, 5.0, 1e-12);
}
