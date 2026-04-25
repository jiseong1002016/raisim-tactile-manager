#include "raisim_tactile/ContactCollector.hpp"

#include <cmath>

#include "raisim/math.hpp"

namespace raisim_tactile {

ContactCollector::ContactCollector(raisim::ArticulatedSystem* articulated_system,
                                   std::size_t target_world_index,
                                   double simulation_dt)
    : articulated_system_(articulated_system),
      target_world_index_(target_world_index),
      simulation_dt_(simulation_dt) {
}

std::vector<ContactSample> ContactCollector::collect() const {
  std::vector<ContactSample> samples;
  if (articulated_system_ == nullptr) return samples;

  const auto& contacts = articulated_system_->getContacts();
  samples.reserve(contacts.size());
  for (const auto& contact : contacts) {
    if (contact.skip()) continue;
    if (contact.getPairObjectIndex() != target_world_index_) continue;

    raisim::Vec<3> impulse_W;
    raisim::matvecmul(contact.getContactFrame(), contact.getImpulse(), impulse_W);
    if (!contact.isObjectA()) impulse_W *= -1.0;

    const Eigen::Vector3d impulse = impulse_W.e();
    if (!impulse.allFinite() || impulse.squaredNorm() < 1.0e-24) continue;

    Eigen::Vector3d normal = contact.getNormal().e();
    if (!contact.isObjectA()) normal = -normal;
    if (!normal.allFinite() || normal.squaredNorm() < 1.0e-24) continue;
    normal.normalize();

    ContactSample sample;
    sample.position_W = contact.getPosition().e();
    sample.impulse_W = impulse;
    sample.normal_W = normal;
    sample.normal_impulse = impulse.dot(normal);
    const Eigen::Vector3d tangent_impulse = impulse - sample.normal_impulse * normal;
    sample.tangential_impulse = tangent_impulse.norm();
    sample.local_body_index = static_cast<int>(contact.getlocalBodyIndex());
    samples.push_back(sample);
  }
  return samples;
}

Eigen::Vector3d ContactCollector::collectBodyForce_W(int local_body_index) const {
  Eigen::Vector3d force_W = Eigen::Vector3d::Zero();
  if (articulated_system_ == nullptr) return force_W;

  const auto& contacts = articulated_system_->getContacts();
  for (const auto& contact : contacts) {
    if (contact.skip()) continue;
    if (contact.getPairObjectIndex() != target_world_index_) continue;
    if (static_cast<int>(contact.getlocalBodyIndex()) != local_body_index) continue;

    raisim::Vec<3> impulse_W;
    raisim::matvecmul(contact.getContactFrame(), contact.getImpulse(), impulse_W);
    if (!contact.isObjectA()) impulse_W *= -1.0;
    force_W += impulse_W.e() / std::max(simulation_dt_, 1.0e-9);
  }
  return force_W;
}

}  // namespace raisim_tactile
