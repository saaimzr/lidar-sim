// Bounding volume hierarchy over a triangle list.
//
// Build: top-down, splitting each node with the surface area heuristic (SAH) evaluated over
// 12 bins of triangle centroids per axis. Query: iterative traversal with an explicit stack,
// visiting the nearer child first so far subtrees are usually skipped.
#pragma once

#include <vector>

#include "lidarsim/geometry.hpp"

namespace lidarsim {

// Optional per-query work counters, used by the benchmark to measure tree quality.
struct TraversalStats {
    long long nodesVisited = 0;
    long long trianglesTested = 0;
};

class Bvh {
public:
    void build(const std::vector<Triangle>& triangles);

    // Nearest hit in (tMin, tMax), or an invalid Hit. Hit::triangle indexes the build() list.
    Hit intersect(const Ray& ray, double tMin, double tMax, TraversalStats* stats = nullptr) const;

    int nodeCount() const { return static_cast<int>(nodes_.size()); }
    int depth() const { return depth_; }

private:
    struct Node {
        AABB box;
        int left = -1;   // interior: left child index; the right child is always left + 1
        int axis = 0;    // interior: split axis (left child holds the smaller centroids)
        int first = 0;   // leaf: first entry in order_
        int count = 0;   // leaf: number of triangles (0 means interior)
    };

    void buildNode(int index, int first, int count, int level);

    const std::vector<Triangle>* tris_ = nullptr;
    std::vector<Node> nodes_;
    std::vector<int> order_;      // triangle indices, grouped so each leaf owns a contiguous range
    std::vector<Vec3> centroid_;  // cached per triangle for the build
    int depth_ = 0;
};

// Reference implementation: test every triangle. Used to validate the BVH and as the benchmark baseline.
Hit intersectBruteForce(const std::vector<Triangle>& triangles, const Ray& ray, double tMin, double tMax);

}  // namespace lidarsim
