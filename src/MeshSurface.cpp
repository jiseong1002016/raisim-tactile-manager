#include "raisim_tactile/MeshSurface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace raisim_tactile {

bool MeshSurface::loadObj(const std::string& obj_path, double scale) {
  clear();

  std::ifstream fin(obj_path);
  if (!fin.is_open()) return false;

  std::vector<Eigen::Vector3d> verts;
  std::vector<std::array<int, 3>> faces;
  std::string line;
  while (std::getline(fin, line)) {
    if (line.size() < 2) continue;
    if (line[0] == 'v' && line[1] == ' ') {
      double x = 0.0, y = 0.0, z = 0.0;
      if (std::sscanf(line.c_str(), "v %lf %lf %lf", &x, &y, &z) == 3) {
        verts.push_back({x, y, z});
      }
    } else if (line[0] == 'f' && line[1] == ' ') {
      int v0 = 0, v1 = 0, v2 = 0, n0 = 0, n1 = 0, n2 = 0;
      if (std::sscanf(line.c_str(), "f %d//%d %d//%d %d//%d",
                      &v0, &n0, &v1, &n1, &v2, &n2) == 6) {
        faces.push_back({v0 - 1, v1 - 1, v2 - 1});
      }
    }
  }

  vertex_count_ = verts.size();
  triangles_.reserve(faces.size());
  for (const auto& f : faces) {
    if (f[0] < 0 || f[1] < 0 || f[2] < 0) continue;
    if (f[0] >= static_cast<int>(verts.size()) ||
        f[1] >= static_cast<int>(verts.size()) ||
        f[2] >= static_cast<int>(verts.size())) {
      continue;
    }

    Triangle tri;
    tri.v0 = verts[f[0]] * scale;
    tri.v1 = verts[f[1]] * scale;
    tri.v2 = verts[f[2]] * scale;
    tri.normal = (tri.v1 - tri.v0).cross(tri.v2 - tri.v0);
    const double len = tri.normal.norm();
    if (len > 1.0e-12) {
      tri.normal /= len;
    } else {
      tri.normal = Eigen::Vector3d::UnitZ();
    }
    triangles_.push_back(tri);
  }

  return !triangles_.empty();
}

void MeshSurface::clear() {
  vertex_count_ = 0;
  triangles_.clear();
}

MeshSurfaceQuery MeshSurface::closestPoint(const Eigen::Vector3d& point) const {
  MeshSurfaceQuery out;
  out.signed_distance = std::numeric_limits<double>::infinity();
  out.abs_distance = std::numeric_limits<double>::infinity();

  for (int i = 0; i < static_cast<int>(triangles_.size()); ++i) {
    Eigen::Vector3d closest;
    const double signed_distance = pointTriangleSignedDistance(point, triangles_[i], closest);
    const double abs_distance = std::abs(signed_distance);
    if (abs_distance < out.abs_distance) {
      out.signed_distance = signed_distance;
      out.abs_distance = abs_distance;
      out.triangle_index = i;
      out.closest_point = closest;
      out.normal = triangles_[i].normal;
    }
  }

  if (out.triangle_index < 0) {
    out.signed_distance = 0.0;
    out.abs_distance = 0.0;
  }
  return out;
}

double MeshSurface::pointTriangleSignedDistance(const Eigen::Vector3d& point,
                                                const Triangle& tri,
                                                Eigen::Vector3d& closest) {
  const Eigen::Vector3d e0 = tri.v1 - tri.v0;
  const Eigen::Vector3d e1 = tri.v2 - tri.v0;
  const Eigen::Vector3d v = point - tri.v0;
  const double d00 = e0.dot(e0);
  const double d01 = e0.dot(e1);
  const double d11 = e1.dot(e1);
  const double d20 = v.dot(e0);
  const double d21 = v.dot(e1);
  const double denom = d00 * d11 - d01 * d01;
  if (std::abs(denom) < 1.0e-15) {
    closest = tri.v0;
    return (point - closest).norm();
  }

  const double s = (d11 * d20 - d01 * d21) / denom;
  const double t = (d00 * d21 - d01 * d20) / denom;
  if (s >= 0.0 && t >= 0.0 && s + t <= 1.0) {
    closest = tri.v0 + s * e0 + t * e1;
  } else {
    auto clampEdge = [](const Eigen::Vector3d& a,
                        const Eigen::Vector3d& b,
                        const Eigen::Vector3d& pt) {
      const Eigen::Vector3d ab = b - a;
      const double tc = std::clamp(
          ab.dot(pt - a) / std::max(ab.squaredNorm(), 1.0e-15), 0.0, 1.0);
      return a + tc * ab;
    };
    const Eigen::Vector3d c0 = clampEdge(tri.v0, tri.v1, point);
    const Eigen::Vector3d c1 = clampEdge(tri.v1, tri.v2, point);
    const Eigen::Vector3d c2 = clampEdge(tri.v2, tri.v0, point);
    const double d0 = (point - c0).squaredNorm();
    const double d1 = (point - c1).squaredNorm();
    const double d2 = (point - c2).squaredNorm();
    if (d0 <= d1 && d0 <= d2) {
      closest = c0;
    } else if (d1 <= d2) {
      closest = c1;
    } else {
      closest = c2;
    }
  }

  const double dist = (point - closest).norm();
  double sign = tri.normal.dot(point - closest) >= 0.0 ? 1.0 : -1.0;
  if (s >= 0.0 && t >= 0.0 && s + t <= 1.0) {
    sign = tri.normal.dot(point - tri.v0) >= 0.0 ? 1.0 : -1.0;
  }
  return sign * dist;
}

}  // namespace raisim_tactile
