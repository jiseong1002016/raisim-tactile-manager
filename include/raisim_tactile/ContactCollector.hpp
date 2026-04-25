#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <vector>

#include "raisim/object/ArticulatedSystem/ArticulatedSystem.hpp"

namespace raisim_tactile {

struct ContactSample {
  Eigen::Vector3d position_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d impulse_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal_W = Eigen::Vector3d::Zero();
  double normal_impulse = 0.0;
  double tangential_impulse = 0.0;
  int local_body_index = -1;
};

class ContactCollector {
 public:
  ContactCollector(raisim::ArticulatedSystem* articulated_system,
                   std::size_t target_world_index,
                   double simulation_dt);

  std::vector<ContactSample> collect() const;
  Eigen::Vector3d collectBodyForce_W(int local_body_index) const;

 private:
  raisim::ArticulatedSystem* articulated_system_ = nullptr;
  std::size_t target_world_index_ = 0;
  double simulation_dt_ = 0.0025;
};

}  // namespace raisim_tactile
