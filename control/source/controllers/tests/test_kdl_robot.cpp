#include "controllers/KDLRobot.h"

#include <iostream>
#include <kdl/frames_io.hpp>

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  controller::KDLRobot robot(controller::KDLRobotTypes::FRANKA_PANDA);

  // in the zero position, the flange should be at position {0.088, 0, 0.926} and 180 deg rotation around X
  KDL::JntArray q(7);
  q.data << 0, 0, 0, 0, 0, 0, 0;
  KDL::Frame frame;

  if (robot.fkPos(q, frame) < 0) {
    std::cout << "fk failed" << std::endl;
    return -1;
  }
  std::cout << "fk result (q -> p): " << std::endl;
  std::cout << q.data << std::endl;
  std::cout << frame << std::endl;

  KDL::JntArray qInit(7);
  qInit.data << 0.1, 0, 0, 0, 0, 0, 0;
  if (robot.ikPos(qInit, frame, q) < 0) {
    std::cout << "ik pos failed" << std::endl;
    return -1;
  }
  std::cout << "ik pos result (qinit, p -> q): " << std::endl;
  std::cout << qInit.data << std::endl;
  std::cout << frame << std::endl;
  std::cout << q.data << std::endl;

  KDL::Twist twist(KDL::Vector(1, 0, 0), KDL::Vector(0, 0, 0));
  KDL::JntArray qdot(7);
  qInit.data << 0, 0, 0, 0, 0, 0, 0;
  if (robot.ikVel(qInit, twist, qdot) < 0) {
    std::cout << "ik vel failed" << std::endl;
  }
  std::cout << "ik vel result (q, v -> qdot): " << std::endl;
  std::cout << qInit.data << std::endl;
  std::cout << twist << std::endl;
  std::cout << qdot.data << std::endl;

}