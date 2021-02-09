#pragma once

#include <Eigen/Eigen>

#include "BaseDS.h"

namespace motion_generator {
class RingDS : public BaseDS {
public:
  RingDS();
  StateRepresentation::CartesianTwist getTwist(const StateRepresentation::CartesianPose& pose) override;

  Eigen::Vector3d center;         // - center of the circle [m]
  Eigen::Quaterniond inclination; // - orientation of the circular plane
  Eigen::Quaterniond defaultPose; // - default orientation of the target frame, relative to the circular frame
  double radius;                  // - circle radius [m]
  double width;                   // - distance around radius where field rotates [m]
  double speed;                   // - desired linear speed when travelling along the circle radius [m/s]
  double fieldStrength;           // - scale factor for desired speed outside of radius + width
  double normalGain;              // - scale factor for the speed normal to the circular plane
  double angularGain;             // - scale factor for angular velocity restitution
private:
  void calculateLinearVelocity(const Eigen::Vector3d& position, Eigen::Vector3d& velocity);
  void calculateAngularVelocity(const Eigen::Quaterniond& orientation, Eigen::Vector3d& velocity);

  Eigen::Vector2d position2d_;
  Eigen::Vector2d velocity2d_;
  double localFieldStrength_;
};
}
