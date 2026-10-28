// Geometry primitives: every expected value here is worked out by hand.
#include <cstdio>
#include <fstream>

#include "lidarsim/geometry.hpp"
#include "lidarsim/mesh.hpp"
#include "test.hpp"

using namespace lidarsim;

// Right triangle in the z = 0 plane with legs along +x and +y.
static const Triangle kTri{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};

TEST(triangle_hit_distance_is_exact) {
    double t = 0;
    // Straight down from 5 m above an interior point: t = 5.
    CHECK(intersectTriangle({{0.5, 0.5, 5}, {0, 0, -1}}, kTri, 0, kInf, t));
    CHECK_NEAR(t, 5.0, 1e-12);
    // Slanted ray: from (0.5, 0.5, 3) towards (0.2, 0.9, 0); t equals that segment's length.
    const Vec3 o{0.5, 0.5, 3}, target{0.2, 0.9, 0};
    CHECK(intersectTriangle({o, normalize(target - o)}, kTri, 0, kInf, t));
    CHECK_NEAR(t, length(target - o), 1e-12);
    // Two-sided: hitting the back face from below also counts.
    CHECK(intersectTriangle({{0.5, 0.5, -2}, {0, 0, 1}}, kTri, 0, kInf, t));
    CHECK_NEAR(t, 2.0, 1e-12);
}

TEST(triangle_misses_and_limits) {
    double t = 0;
    CHECK(!intersectTriangle({{1.5, 1.5, 5}, {0, 0, -1}}, kTri, 0, kInf, t));  // past the hypotenuse
    CHECK(!intersectTriangle({{-0.1, 0.5, 5}, {0, 0, -1}}, kTri, 0, kInf, t)); // left of the x = 0 edge
    CHECK(!intersectTriangle({{0.5, 0.5, 5}, {0, 0, 1}}, kTri, 0, kInf, t));   // pointing away
    CHECK(!intersectTriangle({{0.5, 0.5, 1}, {1, 0, 0}}, kTri, 0, kInf, t));   // parallel to the plane
    CHECK(!intersectTriangle({{0.5, 0.5, 5}, {0, 0, -1}}, kTri, 0, 4.9, t));   // beyond tMax
    CHECK(!intersectTriangle({{0.5, 0.5, 5}, {0, 0, -1}}, kTri, 5.1, kInf, t)); // before tMin
}

TEST(aabb_slab_test) {
    AABB box;
    box.expand({-1, -1, -1});
    box.expand({1, 1, 1});
    auto hits = [&](Vec3 o, Vec3 d, double tMax) {
        d = normalize(d);
        return intersectAABB({o, d}, {1 / d.x, 1 / d.y, 1 / d.z}, box, tMax);
    };
    CHECK(hits({-5, 0, 0}, {1, 0, 0}, kInf));
    CHECK(!hits({-5, 0, 0}, {1, 0, 0}, 3.9));  // box starts at distance 4
    CHECK(!hits({-5, 2, 0}, {1, 0, 0}, kInf)); // passes above
    CHECK(hits({0, 0, 0}, {0.3, -0.2, 0.9}, kInf));  // origin inside the box
    CHECK(hits({-5, -5, -5}, {1, 1, 1}, kInf));       // diagonal through the corner region
}

TEST(pose_round_trip) {
    const Pose pose{{3, -2, 1.5}, 0.7};
    const Vec3 p{1.2, -0.4, 0.9};
    const Vec3 back = pose.toLocal(pose.toWorld(p));
    CHECK_NEAR(back.x, p.x, 1e-12);
    CHECK_NEAR(back.y, p.y, 1e-12);
    CHECK_NEAR(back.z, p.z, 1e-12);
    // A 90-degree yaw maps +x to +y.
    const Vec3 r = Pose{{}, kPi / 2}.rotate({1, 0, 0});
    CHECK_NEAR(r.x, 0, 1e-12);
    CHECK_NEAR(r.y, 1, 1e-12);
}

TEST(mesh_primitives_and_obj_loader) {
    CHECK(makeBox(1, 2, 3).faces.size() == 12);
    CHECK(makeCylinder(1, 2, 16).faces.size() == 16 * 4);
    // Write a unit square as a quad and a triangle-fan polygon, then read it back.
    const std::string path = validationDir() + "/square.obj";
    {
        std::ofstream out(path);
        out << "# test\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1/1 2/2 3/3 4/4\n";
    }
    const Mesh m = loadObj(path);
    CHECK(m.vertices.size() == 4);
    CHECK(m.faces.size() == 2);
    CHECK(m.faces[1][0] == 0 && m.faces[1][1] == 2 && m.faces[1][2] == 3);
    std::remove(path.c_str());
}
