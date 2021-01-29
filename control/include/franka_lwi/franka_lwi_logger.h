#pragma once

#include <ctime>
#include <fstream>
#include <iostream>

#include "franka_lwi_communication_protocol.h"

namespace frankalwi::proto {

class Logger {
private:
  std::ofstream outputFile_;
  int size_ = 0;
  int maxSize_ = 600000;

  template<std::size_t T>
  void writeColumns(const std::array<frankalwi::proto::datatype, T>& data) {
    for (std::size_t i = 0; i < T; ++i) {
      outputFile_ << data.at(i) << ",";
    }
  }
  template<std::size_t T>
  void writeLast(const std::array<frankalwi::proto::datatype, T>& data) {
    for (std::size_t i = 0; i < T - 1; ++i) {
      outputFile_ << data.at(i) << ",";
    }
    outputFile_ << data.at(T - 1) << std::endl;
  }

public:
  Logger(const std::string& filename = "") {
    std::string path = "/tmp/";
    if (filename.empty()) {
      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      std::ostringstream oss;
      oss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
      auto default_name = oss.str() + ".csv";
      outputFile_.open(path + default_name, std::ofstream::out | std::ofstream::trunc);
    } else {
      outputFile_.open(path + filename, std::ofstream::out | std::ofstream::trunc);
    }

    if (!outputFile_.is_open()) {
      std::cerr << "Cannot open output data file, the location might be invalid." << std::endl;
    }
    if (outputFile_.is_open()) {
      outputFile_ << "joint0_pos,joint1_pos,joint2_pos,joint3_pos,joint4_pos,joint5_pos,joint6_pos,";
      outputFile_ << "joint0_vel,joint1_vel,joint2_vel,joint3_vel,joint4_vel,joint5_vel,joint6_vel,";
      outputFile_ << "joint0_eff,joint1_eff,joint2_eff,joint3_eff,joint4_eff,joint5_eff,joint6_eff,";
      outputFile_ << "ee_pos_x,ee_pos_y,ee_pos_z,ee_ori_w,ee_ori_x,ee_ori_y,ee_ori_z,";
      outputFile_ << "ee_twist_lin_x,ee_twist_lin_y,ee_twist_lin_z,ee_twist_ang_x,ee_twist_ang_y,ee_twist_ang_z,";
      outputFile_ << "ee_force_x,ee_force_y,ee_force_z,ee_torque_x,ee_torque_y,ee_torque_z,";
      for (std::size_t column = 0; column < 7; ++column) {
        for (std::size_t row = 0; row < 6; ++row) {
          if (column == 6 && row == 5) {
            outputFile_ << "jac_" + std::to_string(row) + "_" + std::to_string(column) << std::endl;
          } else {
            outputFile_ << "jac_" + std::to_string(row) + "_" + std::to_string(column) + ",";
          }
        }
      }
    }
  }

  void writeLine(const frankalwi::proto::StateMessage<7>& state) {
    if (outputFile_.is_open() && size_ < maxSize_) {
      writeColumns(state.jointPosition.data);
      writeColumns(state.jointVelocity.data);
      writeColumns(state.jointTorque.data);
      writeColumns(frankalwi::proto::vec3DToArray(state.eePose.position));
      writeColumns(frankalwi::proto::quaternionToArray(state.eePose.orientation));
      writeColumns(frankalwi::proto::vec3DToArray(state.eeTwist.linear));
      writeColumns(frankalwi::proto::vec3DToArray(state.eeTwist.angular));
      writeColumns(frankalwi::proto::vec3DToArray(state.eeWrench.linear));
      writeColumns(frankalwi::proto::vec3DToArray(state.eeWrench.angular));
      writeLast(state.jacobian);
      ++size_;
    }
  }
};
}
