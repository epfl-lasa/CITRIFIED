#include "gtest/gtest.h"

#include "controllers/AdaptiveForceFollower.h"

class TestAdaptiveForceFollower : public ::testing::Test {
public:
  TestAdaptiveForceFollower() : force(1, 0, 0), velocity(1, 0, 0) {
    AFF.etol = etol;
    AFF.adaptiveWeight = 1;
    AFF.minDamping = minDamping;
    AFF.maxDamping = maxDamping;
  }
  AdaptiveForceFollower AFF;
  Eigen::Vector3d force;
  Eigen::Vector3d velocity;

  double etol = 0.1;
  double minDamping = 1;
  double maxDamping = 10;
};

TEST_F(TestAdaptiveForceFollower, zero_adaptive_weight) {
  Eigen::Vector3d adaptedVelocity;
  double adaptedDamping;

  AFF.adaptiveWeight = 0;
  AFF.update(force.setRandom(), velocity.setRandom(), adaptedVelocity, adaptedDamping);

  EXPECT_NEAR(adaptedDamping, AFF.defaultDamping, 1e-3);
}

TEST_F(TestAdaptiveForceFollower, zero_velocity) {
  Eigen::Vector3d adaptedVelocity;
  double adaptedDamping;

  velocity = Eigen::Vector3d::Zero();
  force.setRandom();
  AFF.update(force, velocity, adaptedVelocity, adaptedDamping);

  EXPECT_NEAR(adaptedDamping, maxDamping, 1e-3);
  EXPECT_NEAR(adaptedVelocity.normalized().dot(force.normalized()), 1, 1e-3);
  EXPECT_NEAR(adaptedVelocity.norm(), force.norm() / maxDamping, 1e-3);
}

TEST_F(TestAdaptiveForceFollower, zero_force) {
  Eigen::Vector3d adaptedVelocity;
  double adaptedDamping;

  velocity.setRandom();
  force = Eigen::Vector3d::Zero();
  AFF.update(force, velocity, adaptedVelocity, adaptedDamping);

  EXPECT_NEAR(adaptedDamping, minDamping, 1e-3);
  EXPECT_NEAR(adaptedVelocity.norm(), 0, 1e-3);
}

TEST_F(TestAdaptiveForceFollower, parallel_force) {
  Eigen::Vector3d adaptedVelocity;
  double adaptedDamping;

  force = {1, 0, 0};
  velocity = {1, 0, 0};
  AFF.maxDamping = 2 * force.norm() / (etol * velocity.norm());
  AFF.update(force, velocity, adaptedVelocity, adaptedDamping);

  EXPECT_NEAR(adaptedDamping, force.norm() / (etol * velocity.norm()), 1e-3);
  EXPECT_NEAR(adaptedVelocity.norm(), 0, 1e-3);
}

TEST_F(TestAdaptiveForceFollower, orthogonal_force) {
  Eigen::Vector3d adaptedVelocity;
  double adaptedDamping;

  force = {0, 1, 0};
  velocity = {1, 0, 0};
  AFF.update(force, velocity, adaptedVelocity, adaptedDamping);

  EXPECT_NEAR(adaptedDamping, AFF.minDamping, 1e-3);
  EXPECT_NEAR(adaptedVelocity.normalized().dot(force.normalized()), 1, 1e-3);
  EXPECT_NEAR(adaptedVelocity.norm(), force.norm() / AFF.minDamping, 1e-3);
}