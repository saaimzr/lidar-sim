#include "lidarsim/mesh.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace lidarsim {

Mesh makeBox(double sx, double sy, double sz) {
    Mesh m;
    const double hx = sx / 2, hy = sy / 2;
    // Bottom ring (z = 0) then top ring (z = sz), counter-clockwise seen from above.
    m.vertices = {{-hx, -hy, 0}, {hx, -hy, 0}, {hx, hy, 0}, {-hx, hy, 0},
                  {-hx, -hy, sz}, {hx, -hy, sz}, {hx, hy, sz}, {-hx, hy, sz}};
    m.addQuad(0, 3, 2, 1);  // bottom
    m.addQuad(4, 5, 6, 7);  // top
    m.addQuad(0, 1, 5, 4);  // -y side
    m.addQuad(1, 2, 6, 5);  // +x side
    m.addQuad(2, 3, 7, 6);  // +y side
    m.addQuad(3, 0, 4, 7);  // -x side
    return m;
}

Mesh makePlane(double size, int divisions) {
    Mesh m;
    const int n = divisions;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) m.vertices.push_back({size * (double(i) / n - 0.5), size * (double(j) / n - 0.5), 0});
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const int v = j * (n + 1) + i;
            m.addQuad(v, v + 1, v + n + 2, v + n + 1);
        }
    return m;
}

Mesh makeCylinder(double radius, double height, int segments) {
    Mesh m;
    for (int i = 0; i < segments; ++i) {
        const double a = 2 * kPi * i / segments;
        m.vertices.push_back({radius * std::cos(a), radius * std::sin(a), 0});
        m.vertices.push_back({radius * std::cos(a), radius * std::sin(a), height});
    }
    const int bottom = static_cast<int>(m.vertices.size());
    m.vertices.push_back({0, 0, 0});
    m.vertices.push_back({0, 0, height});
    for (int i = 0; i < segments; ++i) {
        const int j = (i + 1) % segments;
        m.addQuad(2 * i, 2 * j, 2 * j + 1, 2 * i + 1);         // side
        m.faces.push_back({bottom, 2 * j, 2 * i});              // bottom cap
        m.faces.push_back({bottom + 1, 2 * i + 1, 2 * j + 1});  // top cap
    }
    return m;
}

Mesh makeSphere(double radius, int rings, int segments) {
    Mesh m;
    for (int r = 0; r <= rings; ++r) {
        const double phi = kPi * r / rings;  // 0 at the north pole
        for (int s = 0; s <= segments; ++s) {
            const double theta = 2 * kPi * s / segments;
            m.vertices.push_back({radius * std::sin(phi) * std::cos(theta),
                                  radius * std::sin(phi) * std::sin(theta),
                                  radius * std::cos(phi)});
        }
    }
    const int row = segments + 1;
    for (int r = 0; r < rings; ++r)
        for (int s = 0; s < segments; ++s)
            m.addQuad(r * row + s, (r + 1) * row + s, (r + 1) * row + s + 1, r * row + s + 1);
    return m;
}

Mesh loadObj(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open " + path);
    Mesh m;
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "v") {
            Vec3 v;
            ss >> v.x >> v.y >> v.z;
            m.vertices.push_back(v);
        } else if (tag == "f") {
            // Tokens look like "7", "7/2" or "7/2/5"; only the vertex index matters. Indices are 1-based.
            std::vector<int> idx;
            std::string tok;
            while (ss >> tok) idx.push_back(std::stoi(tok.substr(0, tok.find('/'))) - 1);
            for (size_t i = 1; i + 1 < idx.size(); ++i) m.faces.push_back({idx[0], idx[i], idx[i + 1]});
        }
    }
    if (m.faces.empty()) throw std::runtime_error("no faces in " + path);
    return m;
}

}  // namespace lidarsim
