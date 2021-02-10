#include "gtest/gtest.h"

#include <kdl/frames_io.hpp>

#include "controllers/KDLRobot.h"

class TestKDLRobot : public ::testing::Test {
public:
  TestKDLRobot() : robot(controller::KDLRobotTypes::FRANKA_PANDA), jointsZeroPosition(robot.dof) {
    KDL::Rotation R;
    R.DoRotX(M_PI);

    jointsZeroPosition.data << 0, 0, 0, 0, 0, 0, 0;
    flangeZeroPose = KDL::Frame(R, KDL::Vector(0.088, 0, 0.926));
  }
  controller::KDLRobot robot;
  double tol = 1e-3;
  KDL::JntArray jointsZeroPosition;
  KDL::Frame flangeZeroPose;
};

TEST_F(TestKDLRobot, zero_position_fk) {
  // in the zero position, the flange should be at position {0.088, 0, 0.926} and 180 deg rotation around X
  KDL::Frame frame;

  EXPECT_GE(robot.fkPos(jointsZeroPosition, frame), 0);

  EXPECT_NEAR(frame.p.x(), flangeZeroPose.p.x(), tol);
  EXPECT_NEAR(frame.p.y(), flangeZeroPose.p.y(), tol);
  EXPECT_NEAR(frame.p.z(), flangeZeroPose.p.z(), tol);

  EXPECT_NEAR(frame.M.GetRot().x(), flangeZeroPose.M.GetRot().x(), tol);
  EXPECT_NEAR(frame.M.GetRot().y(), flangeZeroPose.M.GetRot().y(), tol);
  EXPECT_NEAR(frame.M.GetRot().z(), flangeZeroPose.M.GetRot().z(), tol);
}

TEST_F(TestKDLRobot, zero_position_ik) {
  // Solve the IK from the zero pose frame to the joints.
  KDL::JntArray q(robot.dof), qInit(robot.dof);
  qInit.data << 0.1, 0, 0, 0, 0, 0, 0;

  EXPECT_GE(robot.ikPos(qInit, flangeZeroPose, q), 0);

  for (std::size_t j = 0; j < robot.dof; ++j) {
    EXPECT_NEAR(q.data[j], jointsZeroPosition.data[j], 0.05);
  }
}

TEST_F(TestKDLRobot, zero_position_ik_vel) {
  KDL::Twist twist(KDL::Vector(1, 0, 0), KDL::Vector(0, 0, 0));
  KDL::JntArray qdot(robot.dof), qdotExpected(robot.dof);

  // expected joint velocity at zero pose with a linear velocity of 1 along X
  qdotExpected.data << 0, 10.0/7.0, 0, 0, 0, 10.0/7.0, 0;

  EXPECT_GE(robot.ikVel(jointsZeroPosition, twist, qdot), 0);

  for (std::size_t j = 0; j < robot.dof; ++j) {
    EXPECT_NEAR(qdot.data[j], qdotExpected.data[j], tol);
  }
}