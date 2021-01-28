# pragma once
#include <fstream>

#include "franka_lwi_communication_protocol.h"

namespace frankalwi::logger {

class FrankaLWILogger {
private:
  std::ofstream outputFile_;
public:
  FrankaLWILogger(const std::string& filename) {
    outputFile_.open(filename, std::ofstream::out | std::ofstream::trunc);
    if (!outputFile_.is_open()) {
      std::cerr << "Cannot open output data file, the location might be invalid." << std::endl;
    }
  }

  void writeLine(const frankalwi::proto::StateMessage<7>& state) {
    if (outputFile_.is_open()) {
      for (std::size_t dof = 0; dof < 7; ++dof) {
        outputFile_ << state.jointPosition.data[dof] << ",";
      }
      for (std::size_t dof = 0; dof < 7; ++dof) {
        outputFile_ << state.jointVelocity.data[dof] << ",";
      }
      for (std::size_t dof = 0; dof < 7; ++dof) {
        outputFile_ << state.jointTorque.data[dof] << ",";
      }
      outputFile_ << state.eePose.position.x << ",";
      outputFile_ << state.eePose.position.y << ",";
      outputFile_ << state.eePose.position.z << ",";
      outputFile_ << state.eePose.orientation.w << ",";
      outputFile_ << state.eePose.orientation.x << ",";
      outputFile_ << state.eePose.orientation.y << ",";
      outputFile_ << state.eePose.orientation.z << ",";
      outputFile_ << state.eeTwist.linear.x << ",";
      outputFile_ << state.eeTwist.linear.y << ",";
      outputFile_ << state.eeTwist.linear.z << ",";
      outputFile_ << state.eeTwist.angular.x << ",";
      outputFile_ << state.eeTwist.angular.y << ",";
      outputFile_ << state.eeTwist.angular.z << ",";
      outputFile_ << state.eeWrench.linear.x << ",";
      outputFile_ << state.eeWrench.linear.y << ",";
      outputFile_ << state.eeWrench.linear.z << ",";
      outputFile_ << state.eeWrench.angular.x << ",";
      outputFile_ << state.eeWrench.angular.y << ",";
      outputFile_ << state.eeWrench.angular.z << ",";
      for (std::size_t i = 0; i < 7*6 -1; ++i) {
        outputFile_ << state.jacobian.at(i) << ",";
      }
      outputFile_ << state.jacobian.at(7*6-1) << std::endl;
    }
  }
};
}
