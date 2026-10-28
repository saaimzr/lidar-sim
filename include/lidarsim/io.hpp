// Point cloud output.
//  * KITTI .bin: float32 x, y, z, intensity per point, the format PointPillars and most LiDAR tools read.
//  * CSV: every field including ground truth (true range, label, object id) for analysis.
//  * PLY (ASCII): x, y, z, intensity, label, for viewers such as CloudCompare or MeshLab.
#pragma once

#include <string>
#include <vector>

#include "lidarsim/lidar.hpp"

namespace lidarsim {

void writeKittiBin(const std::string& path, const std::vector<LidarPoint>& points);
std::vector<LidarPoint> readKittiBin(const std::string& path);  // fills x, y, z, intensity only
void writeCsv(const std::string& path, const std::vector<LidarPoint>& points);
void writePly(const std::string& path, const std::vector<LidarPoint>& points);

}  // namespace lidarsim
