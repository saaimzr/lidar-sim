// Small 3D vector and planar pose types used everywhere in the simulator.
//
// Frames: right-handed, z up. A Pose places a frame in the world with a position and a
// yaw (rotation about +z). The LiDAR's own frame is x forward, y left, z up.
#pragma once

#include <cmath>
#include <limits>

namespace lidarsim {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

inline double deg2rad(double deg) { return deg * kPi / 180.0; }

struct Vec3 {
    double x = 0, y = 0, z = 0;

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }

    double operator[](int axis) const { return axis == 0 ? x : axis == 1 ? y : z; }
};

inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(const Vec3& v) { return v / length(v); }
inline Vec3 minv(const Vec3& a, const Vec3& b) { return {std::fmin(a.x, b.x), std::fmin(a.y, b.y), std::fmin(a.z, b.z)}; }
inline Vec3 maxv(const Vec3& a, const Vec3& b) { return {std::fmax(a.x, b.x), std::fmax(a.y, b.y), std::fmax(a.z, b.z)}; }

// Position + heading. Enough for ground vehicles and for placing objects in a scene.
struct Pose {
    Vec3 position;
    double yaw = 0;  // radians, counter-clockwise about +z

    // Rotate a direction from this frame into the world (no translation).
    Vec3 rotate(const Vec3& v) const {
        const double c = std::cos(yaw), s = std::sin(yaw);
        return {c * v.x - s * v.y, s * v.x + c * v.y, v.z};
    }
    // Inverse of rotate(): a world direction expressed in this frame.
    Vec3 unrotate(const Vec3& v) const {
        const double c = std::cos(yaw), s = std::sin(yaw);
        return {c * v.x + s * v.y, -s * v.x + c * v.y, v.z};
    }
    Vec3 toWorld(const Vec3& p) const { return position + rotate(p); }
    Vec3 toLocal(const Vec3& p) const { return unrotate(p - position); }
};

}  // namespace lidarsim
