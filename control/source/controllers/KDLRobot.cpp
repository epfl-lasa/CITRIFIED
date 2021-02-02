#include "controllers/KDLRobot.h"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>

namespace controller {

KDLRobot::KDLRobot(KDLRobotTypes type) {
  auto ZJoint = KDL::Joint(KDL::Joint::RotZ);
  auto FixedJoint = KDL::Joint(KDL::Joint::Fixed);
  chain_ = KDL::Chain();

  switch (type) {
    case FRANKA_PANDA:
      name = "Franka Panda";
      chain_.addSegment(KDL::Segment(FixedJoint, KDL::Frame::DH_Craig1989(0, 0, 0.333, 0)));
      chain_.addSegment(KDL::Segment(ZJoint, KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));
      chain_.addSegment(KDL::Segment(ZJoint, KDL::Frame::DH_Craig1989(0, M_PI_2, 0.316, 0)));
      chain_.addSegment(KDL::Segment(ZJoint, KDL::Frame::DH_Craig1989(0.0825, M_PI_2, 0, 0)));
      chain_.addSegment(KDL::Segment(ZJoint, KDL::Frame::DH_Craig1989(-0.0825, -M_PI_2, 0.384, 0)));
      chain_.addSegment(KDL::Segment(ZJoint, KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));
      chain_.addSegment(KDL::Segment(ZJoint, KDL::Frame::DH_Craig1989(0.088, M_PI_2, 0, 0)));
      chain_.addSegment(KDL::Segment(ZJoint, KDL::Frame::DH_Craig1989(0, 0, 0.107, 0)));

      dof = chain_.getNrOfJoints();
      qLimMin_.resize(dof);
      qLimMax_.resize(dof);
      qOpt_.resize(dof);
      qWeights_.resize(dof);

      qLimMin_.data << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
      qLimMax_.data << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
      qOpt_.data << 0, 0, 0, -M_PI_2, 0, M_PI_2, 0;
      qWeights_.data << 1, 1, 1, 1, 1, 1, 1;
      break;
  }

  fkSolverPos_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  ikSolverVel_ = std::make_unique<KDL::ChainIkSolverVel_pinv_nso>(chain_, qOpt_, qWeights_);
  ikSolverPos_ = std::make_unique<KDL::ChainIkSolverPos_NR>(chain_, *fkSolverPos_, *ikSolverVel_);
  /* IK position with joint limits:
  ikSolverPos_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(chain_, qLimMin_, qLimMax_, *fkSolverPos_, *ikSolverVel_);
  */
}

int KDLRobot::fkPos(const KDL::JntArray& q, KDL::Frame& p) {
  return fkSolverPos_->JntToCart(q, p);
}

int KDLRobot::ikPos(const KDL::JntArray& qInit, const KDL::Frame& p, KDL::JntArray& q) {
  return ikSolverPos_->CartToJnt(qInit, p, q);
}

int KDLRobot::ikVel(const KDL::JntArray& qInit, const KDL::Twist& v, KDL::JntArray& qdot) {
  return ikSolverVel_->CartToJnt(qInit, v, qdot);
}

}