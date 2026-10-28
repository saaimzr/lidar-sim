#include "lidarsim/io.hpp"

#include <fstream>
#include <stdexcept>

namespace lidarsim {

namespace {
std::ofstream openOut(const std::string& path, bool binary) {
    std::ofstream out(path, binary ? std::ios::binary : std::ios::out);
    if (!out) throw std::runtime_error("cannot write " + path);
    return out;
}
}  // namespace

void writeKittiBin(const std::string& path, const std::vector<LidarPoint>& points) {
    auto out = openOut(path, true);
    for (const auto& p : points) {
        const float rec[4] = {p.x, p.y, p.z, p.intensity};
        out.write(reinterpret_cast<const char*>(rec), sizeof rec);
    }
}

std::vector<LidarPoint> readKittiBin(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path);
    std::vector<LidarPoint> points;
    float rec[4];
    while (in.read(reinterpret_cast<char*>(rec), sizeof rec)) {
        LidarPoint p{};
        p.x = rec[0];
        p.y = rec[1];
        p.z = rec[2];
        p.intensity = rec[3];
        points.push_back(p);
    }
    return points;
}

void writeCsv(const std::string& path, const std::vector<LidarPoint>& points) {
    auto out = openOut(path, false);
    out << "x,y,z,intensity,range,true_range,time,channel,azimuth,object,label\n";
    out.precision(7);
    for (const auto& p : points)
        out << p.x << ',' << p.y << ',' << p.z << ',' << p.intensity << ',' << p.range << ',' << p.trueRange << ','
            << p.time << ',' << p.channel << ',' << p.azimuth << ',' << p.object << ',' << labelName(p.label) << '\n';
}

void writePly(const std::string& path, const std::vector<LidarPoint>& points) {
    auto out = openOut(path, false);
    out << "ply\nformat ascii 1.0\nelement vertex " << points.size()
        << "\nproperty float x\nproperty float y\nproperty float z\nproperty float intensity\n"
           "property uchar label\nend_header\n";
    for (const auto& p : points)
        out << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << ' ' << static_cast<int>(p.label) << '\n';
}

}  // namespace lidarsim
