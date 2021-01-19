#pragma once

#include <memory>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

namespace controller {

enum KDLRobotTypes {
  FRANKA_PANDA
};

class KDLRobot {
public:
  explicit KDLRobot(KDLRobotTypes type);

  int fkPos(const KDL::JntArray& q, KDL::Frame& p);
  int ikPos(const KDL::JntArray& qInit, const KDL::Frame& p, KDL::JntArray& q);
  int ikVel(const KDL::JntArray& qInit, const KDL::Twist& v, KDL::JntArray& qdot);

  std::string name;
  std::size_t dof;

private:
  KDL::Chain chain_;
  KDL::JntArray qLimMin_;
  KDL::JntArray qLimMax_;
  KDL::JntArray qOpt_;
  KDL::JntArray qWeights_;
  std::unique_ptr<KDL::ChainFkSolverPos> fkSolverPos_;
  std::unique_ptr<KDL::ChainIkSolverPos> ikSolverPos_;
  std::unique_ptr<KDL::ChainIkSolverVel> ikSolverVel_;
};

}