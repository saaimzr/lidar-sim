#include "lidarsim/bvh.hpp"

#include <algorithm>

namespace lidarsim {

namespace {
constexpr int kBins = 12;
constexpr int kMaxLeafSize = 4;
constexpr int kMaxStack = 64;
constexpr double kTraversalCost = 1.0;  // cost of visiting a node, relative to one triangle test

AABB triangleBounds(const Triangle& t) {
    AABB b;
    b.expand(t.a);
    b.expand(t.b);
    b.expand(t.c);
    return b;
}

int binOf(double c, double lo, double scale) { return std::min(kBins - 1, static_cast<int>((c - lo) * scale)); }
}  // namespace

void Bvh::build(const std::vector<Triangle>& triangles) {
    tris_ = &triangles;
    nodes_.clear();
    depth_ = 0;
    const int n = static_cast<int>(triangles.size());
    order_.resize(n);
    centroid_.resize(n);
    for (int i = 0; i < n; ++i) {
        order_[i] = i;
        centroid_[i] = triangles[i].centroid();
    }
    if (n == 0) return;
    nodes_.reserve(2 * n);  // a binary tree with <= n leaves has < 2n nodes
    nodes_.emplace_back();
    buildNode(0, 0, n, 1);
}

void Bvh::buildNode(int index, int first, int count, int level) {
    depth_ = std::max(depth_, level);

    AABB bounds, centroidBounds;
    for (int i = first; i < first + count; ++i) {
        bounds.expand(triangleBounds((*tris_)[order_[i]]));
        centroidBounds.expand(centroid_[order_[i]]);
    }
    nodes_[index].box = bounds;
    nodes_[index].first = first;
    nodes_[index].count = count;  // a leaf unless we find a worthwhile split below
    if (count <= kMaxLeafSize || level >= kMaxStack - 2) return;

    // For each axis, drop centroids into bins, then sweep the bin boundaries computing the
    // SAH cost area(left)*count(left) + area(right)*count(right). Keep the cheapest boundary.
    double bestCost = kInf;
    int bestAxis = -1, bestBoundary = -1;
    for (int axis = 0; axis < 3; ++axis) {
        const double lo = centroidBounds.lo[axis], hi = centroidBounds.hi[axis];
        if (hi - lo < 1e-12) continue;  // every centroid at the same coordinate: can't split here
        const double scale = kBins / (hi - lo);

        AABB binBox[kBins];
        int binCount[kBins] = {};
        for (int i = first; i < first + count; ++i) {
            const int b = binOf(centroid_[order_[i]][axis], lo, scale);
            binCount[b]++;
            binBox[b].expand(triangleBounds((*tris_)[order_[i]]));
        }

        // Boundary k sits between bins k-1 and k. Prefix sums give both halves in O(bins).
        double leftArea[kBins];
        int leftCount[kBins];
        AABB acc;
        int accCount = 0;
        for (int k = 1; k < kBins; ++k) {
            acc.expand(binBox[k - 1]);
            accCount += binCount[k - 1];
            leftArea[k] = acc.surfaceArea();
            leftCount[k] = accCount;
        }
        acc = AABB();
        accCount = 0;
        for (int k = kBins - 1; k >= 1; --k) {
            acc.expand(binBox[k]);
            accCount += binCount[k];
            if (leftCount[k] == 0 || accCount == 0) continue;
            const double cost = leftArea[k] * leftCount[k] + acc.surfaceArea() * accCount;
            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = axis;
                bestBoundary = k;
            }
        }
    }

    // Split only if it is expected to beat testing every triangle in this node.
    // (Child costs are scaled by the chance a ray through this box also hits the child box.)
    const double splitCost = kTraversalCost + bestCost / bounds.surfaceArea();
    if (bestAxis < 0 || splitCost >= count) return;

    const double lo = centroidBounds.lo[bestAxis];
    const double scale = kBins / (centroidBounds.hi[bestAxis] - lo);
    int* begin = order_.data() + first;
    int* mid = std::partition(begin, begin + count, [&](int tri) {
        return binOf(centroid_[tri][bestAxis], lo, scale) < bestBoundary;
    });
    const int leftCount = static_cast<int>(mid - begin);

    const int left = static_cast<int>(nodes_.size());
    nodes_.emplace_back();  // left child
    nodes_.emplace_back();  // right child, always left + 1
    nodes_[index].left = left;
    nodes_[index].axis = bestAxis;
    nodes_[index].count = 0;  // now an interior node
    buildNode(left, first, leftCount, level + 1);
    buildNode(left + 1, first + leftCount, count - leftCount, level + 1);
}

Hit Bvh::intersect(const Ray& ray, double tMin, double tMax, TraversalStats* stats) const {
    Hit best;
    best.t = tMax;
    if (nodes_.empty()) return Hit{};

    const Vec3 invDir{1.0 / ray.dir.x, 1.0 / ray.dir.y, 1.0 / ray.dir.z};
    int stack[kMaxStack];
    int top = 0;
    stack[top++] = 0;
    while (top > 0) {
        const Node& node = nodes_[stack[--top]];
        if (stats) stats->nodesVisited++;
        if (!intersectAABB(ray, invDir, node.box, best.t)) continue;  // box is beyond the best hit so far
        if (node.count > 0) {
            if (stats) stats->trianglesTested += node.count;
            for (int i = node.first; i < node.first + node.count; ++i) {
                double t;
                if (intersectTriangle(ray, (*tris_)[order_[i]], tMin, best.t, t)) {
                    best.t = t;
                    best.triangle = order_[i];
                }
            }
            continue;
        }
        // Push the far child first so the near child is popped (and tightens best.t) first.
        const bool leftIsNear = ray.dir[node.axis] >= 0;
        stack[top++] = leftIsNear ? node.left + 1 : node.left;
        stack[top++] = leftIsNear ? node.left : node.left + 1;
    }
    if (!best.valid()) best.t = kInf;
    return best;
}

Hit intersectBruteForce(const std::vector<Triangle>& triangles, const Ray& ray, double tMin, double tMax) {
    Hit best;
    best.t = tMax;
    for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
        double t;
        if (intersectTriangle(ray, triangles[i], tMin, best.t, t)) {
            best.t = t;
            best.triangle = i;
        }
    }
    if (!best.valid()) best.t = kInf;
    return best;
}

}  // namespace lidarsim
