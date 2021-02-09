#include "motion_generators/RingDS.h"

namespace motion_generator {

RingDS::RingDS() :
    center(0, 0, 0),
    inclination(1, 0, 0, 0),
    defaultPose(1, 0, 0, 0),
    radius(0.1),
    speed(0.1),
    width(0.05),
    fieldStrength(1),
    normalGain(0),
    angularGain(1),
    localFieldStrength_(fieldStrength) {

}

StateRepresentation::CartesianTwist RingDS::getTwist(const StateRepresentation::CartesianPose& pose) {
  Eigen::Vector3d position = pose.get_position();
  Eigen::Quaterniond orientation = pose.get_orientation();

  position -= center;
  position = inclination.toRotationMatrix().transpose() * position;

  Eigen::Vector3d linearVelocity;
  calculateLinearVelocity(position, linearVelocity);
  linearVelocity = inclination.toRotationMatrix() * linearVelocity;

  Eigen::Vector3d angularVelocity;
  orientation = inclination.conjugate() * orientation;
  calculateAngularVelocity(orientation, angularVelocity);
  angularVelocity = inclination.toRotationMatrix() * angularVelocity;

  StateRepresentation::CartesianTwist twist(pose);
  twist.set_linear_velocity(linearVelocity);
  twist.set_angular_velocity(angularVelocity);

  clampTwist(twist);
  return twist;
}

void RingDS::calculateLinearVelocity(const Eigen::Vector3d& position, Eigen::Vector3d& velocity) {
  velocity2d_.setZero();
  velocity.setZero();

  // get the 2d components of position on the XY plane
  position2d_ = Eigen::Vector2d(position(0), position(1));

  double d = position2d_.norm();
  if (d < 1e-7) {
    return;
  }

  double re = M_PI_2 * (d - radius) / width;
  if (re > M_PI_2) {
    re = M_PI_2;
  } else if (re < -M_PI_2) {
    re = -M_PI_2;
  }

  // calculate the velocity of a point as an orthogonal unit vector, rectified towards the radius based on re
  Eigen::Matrix2d R;
  R << -sin(re), -cos(re), cos(re), -sin(re);
  velocity2d_ = R * position2d_ / d;

  // scale by the nominal speed
  velocity2d_ *= speed;

  // calculate the normal velocity
  double vz = -normalGain * position.z();

  // combine into 3D velocity
  velocity << velocity2d_, vz;

  // calculate the field strength and scale the velocity
  localFieldStrength_ = fieldStrength + (1 - fieldStrength)*cos(re);
  velocity *= localFieldStrength_;
}

void RingDS::calculateAngularVelocity(const Eigen::Quaterniond& orientation, Eigen::Vector3d& velocity) {
  velocity.setZero();

  double theta = -atan2(position2d_.y(), position2d_.x());

  Eigen::Quaterniond qd = Eigen::Quaterniond::Identity();
  qd.w() = sin(theta / 2);
  qd.z() = cos(theta / 2);

  qd = qd * defaultPose;
  if (orientation.dot(qd) < 0) {
    qd.coeffs() = -qd.coeffs();
  }
//  std::cout << theta << "| " << orientation.angularDistance(qd) << std::endl;

  Eigen::Quaterniond deltaQ = qd * orientation.conjugate();
  if (deltaQ.vec().norm() < 1e-7) {
    return;
  }

  //dOmega = 2 * ln (deltaQ)
  Eigen::Quaterniond deltaOmega = Eigen::Quaterniond::Identity();
  deltaOmega.w() = 0;
  double phi = atan2(deltaQ.vec().norm(), deltaQ.w());
  deltaOmega.vec() = 2 * deltaQ.vec() * phi / sin(phi);

  velocity = angularGain * deltaOmega.vec();
  velocity *= localFieldStrength_;

  if (position2d_.norm() < 1e-7 || velocity2d_.norm() < 1e-7) {
    return;
  }
  double projection = position2d_.normalized().dot((position2d_ + velocity2d_).normalized());
  double dThetaZ = 0;
  if (1 - abs(projection) > 1e-7) {
    dThetaZ = acos(projection);
  }
  velocity.z() += dThetaZ;
}

}