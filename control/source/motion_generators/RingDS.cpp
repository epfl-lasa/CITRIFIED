#include "motion_generators/RingDS.h"

namespace motion_generator {

RingDS::RingDS() :
    center(0, 0, 0),
    inclination(1, 0, 0, 0),
    defaultPose(1, 0, 0, 0),
    radius(0.1),
    width(0.05),
    speed(0.1),
    fieldStrength(1),
    normalGain(1),
    angularGain(1),
    localFieldStrength_(fieldStrength) {
  localPosition_.setZero();
  localOrientation_.setIdentity();
  localLinearVelocity_.setZero();
}

/**
 * Calculate and return the 3D linear and angular velocity of the ring dynamical system based on
 * the current pose and DS parameters.
 * The twist is clamped by the min and max linear and angular velocity parameters.
 * @param pose CartesianPose representing the current pose in the parent frame
 * @return CartesianTwist twist of linear and angular velocity in the parent frame
 */
state_representation::CartesianTwist RingDS::getTwist(const state_representation::CartesianPose& pose) {
  updateLocalPose(pose);

  Eigen::Vector3d linearVelocity = inclination.toRotationMatrix() * calculateLocalLinearVelocity();
  Eigen::Vector3d angularVelocity = inclination.toRotationMatrix() * calculateLocalAngularVelocity();

  state_representation::CartesianTwist twist(pose);
  twist.set_linear_velocity(linearVelocity);
  twist.set_angular_velocity(angularVelocity);

  clampTwist(twist);
  return twist;
}

/**
 * Set the local position and orientation in the circle frame from the pose in the parent frame,
 * based on the circle center and inclination.
 * Note: If this class is ported to the dynamical_systems library, the center and inclination members
 * should be removed; the dynamical system itself should be parented to a translated or rotated frame as necessary.
 * @param pose CartesianPose representing the current pose in the parent frame
 */
void RingDS::updateLocalPose(const state_representation::CartesianPose& pose) {
  localPosition_ = inclination.toRotationMatrix().transpose() * (pose.get_position() - center);

  //FIXME: inclination doesn't seem to be working correctly for calculateLocalAngularVelocity.
  // Works fine with null inclination. Needs appropriate tests.
  localOrientation_ = inclination.conjugate() * pose.get_orientation();
}

/**
 * Use the local position in the circle frame to calculate return the 3D linear velocity in the local frame.
 * This function also sets the class members localLinearVelocity_ and localFieldStrength_ required by
 * the function calculateLocalAngularVelocity.
 * @return Local linear velocity
 */
Eigen::Vector3d RingDS::calculateLocalLinearVelocity() {
  localLinearVelocity_ = Eigen::Vector3d::Zero();

  // get the 2d components of position on the XY plane
  Eigen::Vector2d position2d(localPosition_.x(), localPosition_.y());

  double d = position2d.norm();
  if (d < 1e-7) {
    return localLinearVelocity_;
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
  Eigen::Vector2d velocity2d = R * position2d / d;

  // scale by the nominal speed
  velocity2d *= speed;

  // calculate the normal velocity
  double vz = -normalGain * localPosition_.z();

  // combine into 3D velocity
  localLinearVelocity_ << velocity2d, vz;

  // calculate the field strength and scale the velocity
  localFieldStrength_ = fieldStrength + (1 - fieldStrength) * cos(re);
  localLinearVelocity_ *= localFieldStrength_;

  return localLinearVelocity_;
}

/**
 * Use the local orientation and local linear velocity to calculate and return the local angular velocity
 * in the circle frame.
 * @return Local angular velocity
 */
Eigen::Vector3d RingDS::calculateLocalAngularVelocity() {
  Eigen::Vector3d localAngularVelocity = Eigen::Vector3d::Zero();

  double theta = -atan2(localPosition_.y(), localPosition_.x());

  Eigen::Quaterniond qd = Eigen::Quaterniond::Identity();
  qd.w() = sin(theta / 2);
  qd.z() = cos(theta / 2);

  //FIXME: defaultPose does work for a null inclination, but not for an arbitrary inclination.
  // This could be more of a problem of the inclination than anything else.
  // In any case, the usage of defaultPose (the order / direction of rotations) could be made more clear.
  qd = qd * defaultPose;
  if (localOrientation_.dot(qd) < 0) {
    qd.coeffs() = -qd.coeffs();
  }

  Eigen::Quaterniond deltaQ = qd * localOrientation_.conjugate();
  if (deltaQ.vec().norm() < 1e-7) {
    return localAngularVelocity;
  }

  //dOmega = 2 * ln (deltaQ)
  Eigen::Quaterniond deltaOmega = Eigen::Quaterniond::Identity();
  deltaOmega.w() = 0;
  double phi = atan2(deltaQ.vec().norm(), deltaQ.w());
  deltaOmega.vec() = 2 * deltaQ.vec() * phi / sin(phi);

  localAngularVelocity = angularGain * deltaOmega.vec();
  localAngularVelocity *= localFieldStrength_;

  Eigen::Vector2d position2d(localPosition_.x(), localPosition_.y());
  Eigen::Vector2d linearVelocity2d(localLinearVelocity_.x(), localLinearVelocity_.y());
  if (position2d.norm() < 1e-7 || linearVelocity2d.norm() < 1e-7) {
    return localAngularVelocity;
  }

  double projection = position2d.normalized().dot((position2d + linearVelocity2d).normalized());
  double dThetaZ = 0;
  if (1 - abs(projection) > 1e-7) {
    dThetaZ = acos(projection);
  }
  localAngularVelocity.z() += dThetaZ;

  return localAngularVelocity;
}

}