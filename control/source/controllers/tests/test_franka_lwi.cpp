#include "gtest/gtest.h"

#include "franka_lwi/franka_lwi_utils.h"

class TestFrankaLWI : public ::testing::Test {
public:
  TestFrankaLWI() {
    stateMessage.jointPosition.data = jPositions;
    stateMessage.jointVelocity.data = jVelocities;
    stateMessage.jointTorque.data = jTorques;

    stateMessage.eePose = frankalwi::proto::EEPose(pose);
    stateMessage.eeTwist = frankalwi::proto::EETwist(twist);
    stateMessage.eeWrench = frankalwi::proto::EETwist(wrench);
  }
  frankalwi::proto::StateMessage<7> stateMessage{};
  frankalwi::proto::CommandMessage<7> commandMessage{};

  std::array<double, 7> jPositions = {1, 2, 3, 4, 5, 6, 7};
  std::array<double, 7> jVelocities = {11, 12, 13, 14, 15, 16, 17};
  std::array<double, 7> jTorques = {21, 22, 23, 24, 25, 26, 27};

  std::array<double, 7> pose = {1, 2, 3, 0.182574186, 0.365148372, 0.547722558, 0.730296743};
  std::array<double, 6> twist = {1, 2, 3, 4, 5, 6};
  std::array<double, 6> wrench = {11, 12, 13, 14, 15, 16};
};

TEST_F(TestFrankaLWI, ToCartesianPose) {
  state_representation::CartesianPose cartesianPose;

  frankalwi::utils::toCartesianPose(stateMessage, cartesianPose);
  for (std::size_t idx = 0; idx < 7; ++idx) {
    EXPECT_NEAR(cartesianPose.get_pose()(idx), pose[idx], 1e-9);
  }
}

TEST_F(TestFrankaLWI, ToCartesianState) {
  state_representation::CartesianState cartesianState;

  frankalwi::utils::toCartesianState(stateMessage, cartesianState);
  for (std::size_t idx = 0; idx < 7; ++idx) {
    EXPECT_NEAR(cartesianState.get_pose()(idx), pose[idx], 1e-9);
    if (idx < 6) {
      EXPECT_NEAR(cartesianState.get_twist()(idx), twist[idx], 1e-9);
      EXPECT_NEAR(cartesianState.get_wrench()(idx), wrench[idx], 1e-9);
    }
  }
}

TEST_F(TestFrankaLWI, ToJointState) {
  state_representation::JointState jointState("franka", 7);

  frankalwi::utils::toJointState(stateMessage, jointState);
  for (std::size_t idx = 0; idx < 7; ++idx) {
    EXPECT_NEAR(jointState.get_positions()(idx), jPositions[idx], 1e-9);
    EXPECT_NEAR(jointState.get_velocities()(idx), jVelocities[idx], 1e-9);
    EXPECT_NEAR(jointState.get_torques()(idx), jTorques[idx], 1e-9);
  }
}

TEST_F(TestFrankaLWI, ToJacobian) {
  for (std::size_t idx = 0; idx < 42; ++idx) {
    stateMessage.jacobian[idx] = idx;
  }
  state_representation::Jacobian jacobian("franka", 7);
  frankalwi::utils::toJacobian(stateMessage.jacobian, jacobian);
  for (std::size_t idx = 0; idx < 42; ++idx) {
    EXPECT_NEAR(jacobian.data()(idx % 6, floor(idx / 6)), idx, 1e-9);
  }
}

TEST_F(TestFrankaLWI, FromJointTorques) {
  state_representation::JointState jointState("franka", 7);
  jointState.set_torques(std::vector<double>(jTorques.begin(), jTorques.end()));

  frankalwi::utils::fromJointTorque(jointState, commandMessage);
  for (std::size_t idx = 0; idx < 7; ++idx) {
    EXPECT_NEAR(commandMessage.jointTorque[idx], jTorques[idx], 1e-9);
  }
}