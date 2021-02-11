#pragma once

#include <Eigen/Eigen>

class AdaptiveForceFollower {
public:
  void update(const Eigen::Vector3d& desiredForce,
              const Eigen::Vector3d& desiredVelocity,
              Eigen::Vector3d& adaptiveVelocity,
              double& damping);

  double etol = 0.1;
  double defaultDamping = 1;
  double adaptiveWeight = 0.5;

  double minDamping = 1;
  double maxDamping = 10;

private:
  double clampDamping(double damping) const;

};