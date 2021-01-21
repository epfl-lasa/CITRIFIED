#include "motion_generators/RingDS.h"

namespace motion_generator {
static double clamp(double val, const double min, const double max) {
  if (val < min) {
    val = min;
  } else if (val > max) {
    val = max;
  }
  return val;
}

RingDS::RingDS() :
    center(0, 0, 0),
    inclination(1, 0, 0, 0),
    radius(0.1),
    speed(0.1),
    width(0.05),
    fieldStrength(1),
    normalGain(0),
    angularGain(1) {

}

StateRepresentation::CartesianTwist RingDS::getTwist(const StateRepresentation::CartesianPose& pose) {
  Eigen::Vector3d position = pose.get_position();

  position -= center;
  position = inclination.toRotationMatrix().transpose() * position;

  Eigen::Vector3d linearVelocity;
  calculateLinearVelocity(position, linearVelocity);
  linearVelocity = inclination.toRotationMatrix() * linearVelocity;

  Eigen::Vector3d angularVelocity;
  calculateAngularVelocity(position, angularVelocity);

  StateRepresentation::CartesianTwist twist(pose);
  twist.set_linear_velocity(linearVelocity);
  twist.set_angular_velocity(angularVelocity);

  return twist;
}

void RingDS::calculateLinearVelocity(const Eigen::Vector3d& position, Eigen::Vector3d& velocity) {
  velocity.setZero();

  // get the 2d components of position on the XY plane
  Eigen::Vector2d position2d(position(0), position(1));

  double d = position2d.norm();
  if (d < 1e-7) {
    return;
  }

  double re = M_PI_2 * (d - radius) / width;

  // calculate the velocity of a point as an orthogonal unit vector, rectified towards the radius based on re
  Eigen::Matrix2d R;
  R << -sin(re), -cos(re), cos(re), -sin(re);
  Eigen::Vector2d velocity2d = R * position2d / d;

  // scale by the nominal speed
  velocity2d *= speed;

  // calculate the normal velocity
  double vz = -normalGain * position.z();

  // combine into 3D velocity
  velocity << velocity2d, vz;

  // calculate the field strength and scale the velocity
  double fe = fieldStrength + (1 - fieldStrength)*cos(re);
  velocity *= fe;
}

void RingDS::calculateAngularVelocity(const Eigen::Quaterniond& orientation, Eigen::Vector3d& velocity) {
  velocity.setZero();

  // TODO: recycle these from linear velocity calculation as private members
  Eigen::Vector2d position2d;
  Eigen::Vector2d velocity2d;

  double theta = atan2(position2d.y(), position2d.x());

  Eigen::Quaterniond deltaQ = Eigen::Quaterniond::Identity();
  deltaQ.w() = cos(theta / 2);
  deltaQ.z() = sin(theta / 2);

  Eigen::Quaterniond dq = deltaQ * orientation.conjugate();
  if (dq.vec().norm() < 1e-7) {
    return;
  }

  //dOmega = 2 * ln (dq)
  Eigen::Quaterniond deltaOmega = Eigen::Quaterniond::Identity();
  deltaOmega.w() = 0;
  double phi = atan2(deltaQ.vec().norm(), deltaQ.w());
  deltaOmega.vec() = 2 * deltaQ.vec() * phi / sin(phi);

  velocity = angularGain * deltaOmega.vec();

  // calculate angular velocity around local Z (TODO)
//  \dot{\theta}_z = \arccos{
//    \frac{\bm{p}_{2D} \cdot \left( \bm{p}_{2D} + \dot{\bm{p}}_{2D} \right) }
//    {d * \lVert \bm{p}_{2D} + \dot{\bm{p}}_{2D} \lVert}
//  }
  double dThetaZ = 0; //

  velocity.z() += dThetaZ;
}

}