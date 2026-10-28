// A scene is a set of objects (mesh + pose + material + label) flattened into one world-space
// triangle list with a BVH over it. Per-triangle lookups give the material at a hit.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lidarsim/bvh.hpp"
#include "lidarsim/mesh.hpp"

namespace lidarsim {

// Semantic classes written alongside every point (ground truth for perception training).
enum class Label : uint8_t { None = 0, Ground, Building, Vehicle, Pole, Pedestrian, Wall, Fog };
const char* labelName(Label label);

struct Object {
    Mesh mesh;
    Pose pose;
    double reflectivity = 0.5;  // diffuse reflectance in [0, 1]
    Label label = Label::None;
};

class Scene {
public:
    void add(Object object);
    // Flattens all objects into world space and builds the BVH. Call after the last add().
    void build();

    Hit intersect(const Ray& ray, double tMin, double tMax, TraversalStats* stats = nullptr) const {
        return bvh_.intersect(ray, tMin, tMax, stats);
    }

    const std::vector<Triangle>& triangles() const { return triangles_; }
    const Bvh& bvh() const { return bvh_; }
    const Object& object(int index) const { return objects_[index]; }
    const Object& objectOf(int triangle) const { return objects_[objectIndex_[triangle]]; }
    int objectIndexOf(int triangle) const { return objectIndex_[triangle]; }
    Vec3 normalOf(int triangle) const { return normals_[triangle]; }

private:
    std::vector<Object> objects_;
    std::vector<Triangle> triangles_;
    std::vector<int> objectIndex_;  // triangle -> object
    std::vector<Vec3> normals_;     // triangle -> unit normal (precomputed)
    Bvh bvh_;
};

// Procedural urban street: road plane, building blocks, parked cars, poles and pedestrians.
// `detail` scales the tessellation of round objects (more triangles for benchmarking).
// `groundDivisions` splits the 200 m ground into cells; 1 = two giant triangles (kept as an option
// only so the benchmark can show why splitting matters).
Scene makeStreetScene(uint64_t seed, int detail = 1, int groundDivisions = 100);
// Closed axis-aligned box room with inner size (sx, sy, sz), floor at z = 0, centred on x = y = 0.
// Every surface has the given reflectivity. Used for analytic range validation.
Scene makeRoomScene(double sx, double sy, double sz, double reflectivity);

}  // namespace lidarsim
