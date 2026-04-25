#include <cassert>
#include <iostream>
#include <string>

#include "raisim/World.hpp"
#include "raisim_tactile/ContactCollector.hpp"

int main() {
#ifndef RAISIM_TACTILE_EXAMPLE_GRIPPER_URDF
#error "RAISIM_TACTILE_EXAMPLE_GRIPPER_URDF must be defined"
#endif

  raisim::World world;
  world.setTimeStep(0.001);
  raisim::Vec<3> gravity;
  gravity = {0.0, 0.0, -9.81};
  world.setGravity(gravity);

  auto* ground = world.addGround();
  ground->setName("ground");

  auto* gripper = world.addArticulatedSystem(RAISIM_TACTILE_EXAMPLE_GRIPPER_URDF);
  Eigen::VectorXd gc = Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(gripper->getGeneralizedCoordinateDim()));
  if (gc.size() >= 7) {
    gc[0] = 0.0;
    gc[1] = 0.0;
    gc[2] = 0.15;
    gc[3] = 1.0;
    gc[4] = 0.0;
    gc[5] = 0.0;
    gc[6] = 0.0;
  }
  gripper->setGeneralizedCoordinate(gc);
  gripper->setGeneralizedVelocity(Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(gripper->getGeneralizedVelocityDim())));

  for (int i = 0; i < 1000; ++i) world.integrate();

  raisim_tactile::ContactCollector collector(
      gripper, ground->getIndexInWorld(), world.getTimeStep());
  const auto samples = collector.collect();

  Eigen::Vector3d summed_force = Eigen::Vector3d::Zero();
  for (const auto& sample : samples) {
    summed_force += sample.impulse_W / world.getTimeStep();
  }

  assert(!samples.empty());
  assert(summed_force.allFinite());
  assert(summed_force.norm() > 0.0);

  std::cout << "contact_collector_world OK samples=" << samples.size()
            << " force_norm=" << summed_force.norm() << "\n";
  return 0;
}
