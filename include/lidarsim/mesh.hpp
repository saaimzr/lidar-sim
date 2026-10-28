// Triangle meshes: a few procedural primitives plus a minimal OBJ loader.
// Every primitive is built in its own local frame and sits on z = 0 unless noted.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "lidarsim/math.hpp"

namespace lidarsim {

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<std::array<int, 3>> faces;  // indices into `vertices`

    void addQuad(int a, int b, int c, int d) {  // two triangles: (a,b,c) and (a,c,d)
        faces.push_back({a, b, c});
        faces.push_back({a, c, d});
    }
};

// Axis-aligned box with its base centred on the origin: x in [-sx/2, sx/2], y likewise, z in [0, sz].
Mesh makeBox(double sx, double sy, double sz);
// Flat square on z = 0, centred on the origin, split into divisions x divisions cells.
// Large scenes should split the ground: one huge triangle has a bounding box covering the whole
// scene, which drags giant boxes deep into the BVH and slows every ray.
Mesh makePlane(double size, int divisions = 1);
// Closed cylinder standing on z = 0.
Mesh makeCylinder(double radius, double height, int segments);
// UV sphere centred on the origin.
Mesh makeSphere(double radius, int rings, int segments);
// Wavefront OBJ: reads `v` and `f` lines only (polygons are fan-triangulated). Throws on failure.
Mesh loadObj(const std::string& path);

}  // namespace lidarsim
