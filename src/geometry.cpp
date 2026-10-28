#include "lidarsim/geometry.hpp"

#include <algorithm>

namespace lidarsim {

bool intersectTriangle(const Ray& ray, const Triangle& tri, double tMin, double tMax, double& t) {
    // Solve origin + t*dir = a + u*(b-a) + v*(c-a) with Cramer's rule.
    // kEdgeTol widens the triangle by a hair so rounding can't open a crack along an edge shared
    // by two triangles (a ray exactly on that edge would otherwise be able to miss both).
    constexpr double kEdgeTol = 1e-10;
    const Vec3 e1 = tri.b - tri.a;
    const Vec3 e2 = tri.c - tri.a;
    const Vec3 p = cross(ray.dir, e2);
    const double det = dot(e1, p);
    if (std::fabs(det) < 1e-12) return false;  // ray parallel to the triangle's plane

    const double invDet = 1.0 / det;
    const Vec3 s = ray.origin - tri.a;
    const double u = dot(s, p) * invDet;
    if (u < -kEdgeTol || u > 1.0 + kEdgeTol) return false;

    const Vec3 q = cross(s, e1);
    const double v = dot(ray.dir, q) * invDet;
    if (v < -kEdgeTol || u + v > 1.0 + kEdgeTol) return false;

    const double hitT = dot(e2, q) * invDet;
    if (hitT <= tMin || hitT >= tMax) return false;
    t = hitT;
    return true;
}

bool intersectAABB(const Ray& ray, const Vec3& invDir, const AABB& box, double tMax) {
    // Intersect the ray with the three pairs of parallel planes; the box is hit if the
    // intervals overlap. Division by a zero direction component gives +/-inf, which works out.
    double tEnter = 0.0, tExit = tMax;
    for (int axis = 0; axis < 3; ++axis) {
        double t0 = (box.lo[axis] - ray.origin[axis]) * invDir[axis];
        double t1 = (box.hi[axis] - ray.origin[axis]) * invDir[axis];
        if (t0 > t1) std::swap(t0, t1);
        tEnter = std::max(tEnter, t0);
        tExit = std::min(tExit, t1);
        if (tEnter > tExit) return false;
    }
    return true;
}

}  // namespace lidarsim
