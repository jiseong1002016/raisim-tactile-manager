#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <string>
#include <vector>

namespace raisim_tactile {

struct MeshSurfaceQuery {
  double signed_distance = 0.0;
  double abs_distance = 0.0;
  int triangle_index = -1;
  Eigen::Vector3d closest_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
};

class MeshSurface {
 public:
  bool loadObj(const std::string& obj_path, double scale);
  void clear();

  bool empty() const { return triangles_.empty(); }
  std::size_t triangleCount() const { return triangles_.size(); }
  std::size_t vertexCount() const { return vertex_count_; }

  MeshSurfaceQuery closestPoint(const Eigen::Vector3d& point) const;

 private:
  struct Triangle {
    Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d v1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d v2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  };

  static double pointTriangleSignedDistance(const Eigen::Vector3d& point,
                                            const Triangle& tri,
                                            Eigen::Vector3d& closest);

  std::size_t vertex_count_ = 0;
  std::vector<Triangle> triangles_;
};

}  // namespace raisim_tactile
