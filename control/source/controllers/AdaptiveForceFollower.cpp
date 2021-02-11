#include "controllers/AdaptiveForceFollower.h"

void AdaptiveForceFollower::update(const Eigen::Vector3d& desiredForce,
                                   const Eigen::Vector3d& desiredVelocity,
                                   Eigen::Vector3d& adaptiveVelocity,
                                   double& damping) {
  double f1 = desiredForce.norm();
  double da = maxDamping;
  if (desiredVelocity.norm() > 1e-7) {
    if (desiredForce.norm() > 1e-7) {
      f1 = desiredForce.dot(desiredVelocity.normalized());
    }
    da = f1 / (etol * desiredVelocity.norm());
  }

  damping = clampDamping((1 - adaptiveWeight) * defaultDamping + adaptiveWeight * da);

  Eigen::Vector3d fp = desiredForce - f1 * desiredVelocity.normalized();
  adaptiveVelocity = fp / damping;
}

double AdaptiveForceFollower::clampDamping(const double damping) const {
  double clampedDamp = damping > minDamping ? damping : minDamping;
  return clampedDamp < maxDamping ? clampedDamp : maxDamping;
}