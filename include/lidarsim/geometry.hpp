// Rays, axis-aligned boxes and the two intersection tests the ray tracer is built on.
#pragma once

#include "lidarsim/math.hpp"

namespace lidarsim {

struct Ray {
    Vec3 origin;
    Vec3 dir;  // unit length
};

struct Triangle {
    Vec3 a, b, c;
    Vec3 normal() const { return normalize(cross(b - a, c - a)); }
    Vec3 centroid() const { return (a + b + c) / 3.0; }
};

// Result of a nearest-hit query. `triangle` indexes the scene's triangle list.
struct Hit {
    double t = kInf;
    int triangle = -1;
    bool valid() const { return triangle >= 0; }
};

struct AABB {
    Vec3 lo{kInf, kInf, kInf};
    Vec3 hi{-kInf, -kInf, -kInf};

    void expand(const Vec3& p) { lo = minv(lo, p); hi = maxv(hi, p); }
    void expand(const AABB& b) { lo = minv(lo, b.lo); hi = maxv(hi, b.hi); }
    bool empty() const { return lo.x > hi.x; }
    double surfaceArea() const {
        if (empty()) return 0;
        const Vec3 d = hi - lo;
        return 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
    }
};

// Möller–Trumbore ray/triangle test. Two-sided, because a LiDAR beam returns from either face.
// On a hit inside (tMin, tMax) writes the distance to `t` and returns true.
bool intersectTriangle(const Ray& ray, const Triangle& tri, double tMin, double tMax, double& t);

// Slab test: does the ray enter `box` before distance tMax? `invDir` = 1/ray.dir per axis
// (precomputed once per ray, since the BVH calls this for many boxes).
bool intersectAABB(const Ray& ray, const Vec3& invDir, const AABB& box, double tMax);

}  // namespace lidarsim
