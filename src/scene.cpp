#include "lidarsim/scene.hpp"

#include "lidarsim/rng.hpp"

namespace lidarsim {

const char* labelName(Label label) {
    switch (label) {
        case Label::Ground: return "ground";
        case Label::Building: return "building";
        case Label::Vehicle: return "vehicle";
        case Label::Pole: return "pole";
        case Label::Pedestrian: return "pedestrian";
        case Label::Wall: return "wall";
        case Label::Fog: return "fog";
        default: return "none";
    }
}

void Scene::add(Object object) { objects_.push_back(std::move(object)); }

void Scene::build() {
    triangles_.clear();
    objectIndex_.clear();
    normals_.clear();
    for (int o = 0; o < static_cast<int>(objects_.size()); ++o) {
        const Object& obj = objects_[o];
        for (const auto& f : obj.mesh.faces) {
            Triangle t{obj.pose.toWorld(obj.mesh.vertices[f[0]]), obj.pose.toWorld(obj.mesh.vertices[f[1]]),
                       obj.pose.toWorld(obj.mesh.vertices[f[2]])};
            triangles_.push_back(t);
            objectIndex_.push_back(o);
            normals_.push_back(t.normal());
        }
    }
    bvh_.build(triangles_);
}

Scene makeStreetScene(uint64_t seed, int detail, int groundDivisions) {
    Scene scene;
    Rng rng(seed, 0, 0, 0);

    // A 200 m square of road and pavement.
    scene.add({makePlane(200, groundDivisions), Pose{}, 0.25, Label::Ground});  // default: 2 m cells (see makePlane)

    // Two rows of buildings either side of a 20 m wide street running along x.
    for (int side = -1; side <= 1; side += 2) {
        double x = -90;
        while (x < 90) {
            const double w = rng.uniform(8, 18), d = rng.uniform(10, 20), h = rng.uniform(6, 30);
            const double y = side * (10 + 4 + d / 2);  // 4 m pavement between road edge and facade
            scene.add({makeBox(w, d, h), Pose{{x + w / 2, y, 0}, 0}, rng.uniform(0.3, 0.7), Label::Building});
            x += w + rng.uniform(1, 4);
        }
    }

    // Parked cars along both kerbs, slightly jittered in heading.
    for (int side = -1; side <= 1; side += 2) {
        for (double x = -80; x < 80; x += rng.uniform(7, 12)) {
            if (rng.uniform() < 0.3) continue;  // gaps between cars
            const Pose pose{{x, side * 7.5, 0}, rng.uniform(-0.05, 0.05)};
            Object body{makeBox(4.5, 1.8, 1.0), pose, rng.uniform(0.2, 0.9), Label::Vehicle};
            body.pose.position.z = 0.3;  // ground clearance
            scene.add(body);
            Object cabin{makeBox(2.4, 1.6, 0.6), pose, body.reflectivity, Label::Vehicle};
            cabin.pose.position.z = 1.3;
            scene.add(cabin);
            for (int w = 0; w < 4; ++w) {  // wheels: low-poly spheres, a cheap stand-in that adds curved geometry
                const Vec3 offset{w < 2 ? 1.4 : -1.4, (w % 2 ? 0.8 : -0.8), 0.3};
                Object wheel{makeSphere(0.32, 6 * detail, 10 * detail), Pose{pose.toWorld(offset), pose.yaw}, 0.1,
                             Label::Vehicle};
                scene.add(wheel);
            }
        }
    }

    // Street lights every 15 m on the pavement, and a few pedestrians (cylinder body + sphere head).
    for (double x = -85; x < 90; x += 15) {
        for (int side = -1; side <= 1; side += 2) {
            scene.add({makeCylinder(0.12, 7, 12 * detail), Pose{{x, side * 11.5, 0}, 0}, 0.6, Label::Pole});
        }
    }
    for (int i = 0; i < 24; ++i) {
        // Separate statements: the order of two calls inside one expression is unspecified in C++.
        const double x = rng.uniform(-80, 80);
        const double side = rng.uniform() < 0.5 ? -1.0 : 1.0;
        const double y = side * rng.uniform(10.5, 13.5);
        scene.add({makeCylinder(0.25, 1.45, 12 * detail), Pose{{x, y, 0}, 0}, 0.4, Label::Pedestrian});
        scene.add({makeSphere(0.12, 8 * detail, 12 * detail), Pose{{x, y, 1.6}, 0}, 0.4, Label::Pedestrian});
    }

    scene.build();
    return scene;
}

Scene makeRoomScene(double sx, double sy, double sz, double reflectivity) {
    Scene scene;
    scene.add({makeBox(sx, sy, sz), Pose{}, reflectivity, Label::Wall});
    scene.build();
    return scene;
}

}  // namespace lidarsim
