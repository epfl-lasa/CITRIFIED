#include <yaml-cpp/yaml.h>

#include <dynamical_systems/Linear.hpp>

#include "controllers/TwistController.h"
#include "sensors/ForceTorqueSensor.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "sensors/RigidBodyTracker.h"
#include "filters/DigitalButterworth.h"
#include "network/interfaces.h"
#include "logger/JSONLogger.h"

#define RB_ID_ROBOT_BASE 1
#define RB_ID_TASK_BASE 2

using namespace state_representation;

enum ProbeState {
  APPROACH,
  CALIBRATION,
  TOUCH,
  FINISH
};

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  YAML::Node params = YAML::LoadFile(std::string(TRIAL_CONFIGURATION_DIR) + "surface_probe_parameters.yaml");

  // Configure linear dynamical system
  CartesianPose center("center", "task");
  center.set_position(params["attractor"]["position"]["x"].as<double>(),
                      params["attractor"]["position"]["y"].as<double>(),
                      params["attractor"]["position"]["z"].as<double>());
  center.set_orientation(Eigen::Quaterniond(params["attractor"]["orientation"]["w"].as<double>(),
                                            params["attractor"]["orientation"]["x"].as<double>(),
                                            params["attractor"]["orientation"]["y"].as<double>(),
                                            params["attractor"]["orientation"]["z"].as<double>()));
  auto linearGain = params["attractor"]["gains"]["linear"].as<double>();
  auto angularGain = params["attractor"]["gains"]["angular"].as<double>();
  std::vector<double> gains = {linearGain, linearGain, linearGain, angularGain, angularGain, angularGain};
  double zVelocity = 0;

  dynamical_systems::Linear<CartesianState> pointDS(center, gains);
  pointDS.set_base_frame(CartesianState::Identity("task", "task"));

  // set up controller
  controllers::TwistController ctrl(params["default"]["d1"].as<double>(),
                                    params["default"]["d2"].as<double>(),
                                    params["default"]["ak"].as<double>(),
                                    params["default"]["ad"].as<double>());

  // set up logger
  logger::JSONLogger jsonLogger(params["experiment_prefix"].as<std::string>() + params["surface"].as<std::string>() + "_surface.json");
  jsonLogger.addMetaData(params["surface"].as<std::string>());

  // set up optitrack
  sensors::RigidBodyTracker optitracker;
  optitracker.start();

  // set up FT sensor
  sensors::ToolSpec tool = {
      .centerOfMass = Eigen::Vector3d(0, 0, 0.02),
      .mass = 0.07
  };
  sensors::ForceTorqueSensor ft_sensor("ft_sensor", "128.178.145.248", 100, tool);

  // set up filter
  filter::DigitalButterworth wrenchFilter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 6);

  // set up robot connection
  network::Interface franka(network::InterfaceType::FRANKA_LWI);
  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  // prepare all state objects
  CartesianState eeInRobot("ee", "robot");
  CartesianState taskInOptitrack("task", "optitrack");
  CartesianState robotInOptitrack("robot", "optitrack");
  CartesianWrench ftWrenchInRobot("ft_sensor", "robot");
  CartesianWrench ftWrenchInRobotFilt = ftWrenchInRobot;

  state_representation::Jacobian jacobian("robot", 7, "ee", "robot");

  // wait for optitrack data
  std::cout << "Waiting for optitrack robot base and task base state..." << std::endl;
  while (!optitracker.getState(robotInOptitrack, RB_ID_ROBOT_BASE)
      || !optitracker.getState(taskInOptitrack, RB_ID_TASK_BASE)) {}

  std::cout << "Optitrack ready" << std::endl;

  jsonLogger.addBody(logger::MessageType::STATIC, robotInOptitrack);
  jsonLogger.addBody(logger::MessageType::STATIC, taskInOptitrack);
  jsonLogger.write();

  // set up touch grid
  std::vector<double> x_offsets, y_offsets;
  int x_index = 0, y_index = 0;
  for (int iteration = 0; iteration < params["grid"]["resolution"].as<double>(); ++iteration) {
    double weight = (iteration / (params["grid"]["resolution"].as<double>() - 1));
    x_offsets.push_back(weight * params["grid"]["x_range"].as<double>() - (1 - weight) * params["grid"]["x_range"].as<double>());
    y_offsets.push_back(weight * params["grid"]["y_range"].as<double>() - (1 - weight) * params["grid"]["y_range"].as<double>());
  }

  // start main control loop
  int iterations = 0;
  auto frequencyTimer = std::chrono::system_clock::now();
  auto pauseTimer = std::chrono::system_clock::now();
  ProbeState probeState = ProbeState::APPROACH;
  while (franka.receive(state)) {
    //  update states
    frankalwi::utils::toCartesianState(state, eeInRobot);
    frankalwi::utils::toJacobian(state.jacobian, jacobian);

    // update optitrack states
    optitracker.getState(robotInOptitrack, RB_ID_ROBOT_BASE);
    optitracker.getState(taskInOptitrack, RB_ID_TASK_BASE);
    auto taskInRobot = robotInOptitrack.inverse() * taskInOptitrack;
    auto eeInTask = taskInRobot.inverse() * eeInRobot;
    pointDS.set_base_frame(taskInRobot);

    // update ft wrench
    if (ft_sensor.biasOK()) {
      ft_sensor.readContactWrench(ftWrenchInRobot, eeInRobot.get_orientation().toRotationMatrix());
    }

    // filter data
    ftWrenchInRobotFilt.set_wrench(wrenchFilter.computeFilterOutput(ftWrenchInRobot.get_wrench()));

    if (ftWrenchInRobot.get_force().norm() > params["default"]["max_force"].as<double>()) {
      std::cout << "Max force exceeded" << std::endl;

      probeState = FINISH;
      std::cout << "### FINISHING" << std::endl;
      break;
    }

    // get the current grid position
    CartesianPose attractor = center;
    Eigen::Vector3d gridPosition = attractor.get_position();
    gridPosition.x() += x_offsets[x_index];
    gridPosition.y() += y_offsets[y_index];
    attractor.set_position(gridPosition);
    pointDS.set_attractor(attractor);

    switch (probeState) {
      case APPROACH:
        if ((eeInTask.get_position() - gridPosition).norm() < 0.01) {
          if (!ft_sensor.biasOK()) {
            probeState = CALIBRATION;
            std::cout << "### STARTING CALIBRATION PHASE" << std::endl;
            break;
          }

          gains[2] = 0;
          pointDS.set_gain(gains);
          zVelocity = -params["touch"]["speed"].as<double>();

          // set the controller to specific insertion phase gains
          ctrl.set_linear_damping(params["touch"]["d1"].as<double>(), params["touch"]["d2"].as<double>());
          ctrl.angular_stiffness = params["touch"]["ak"].as<double>();
          ctrl.angular_damping = params["touch"]["ad"].as<double>();

          probeState = TOUCH;
          std::cout << "### STARTING TOUCH PHASE" << std::endl;
        }
        break;
      case CALIBRATION:
        if (ft_sensor.computeBias(eeInRobot.get_orientation().toRotationMatrix(), 2000)) {

          gains[2] = 0;
          pointDS.set_gain(gains);
          zVelocity = -params["touch"]["speed"].as<double>();

          // set the controller to specific insertion phase gains
          ctrl.set_linear_damping(params["touch"]["d1"].as<double>(), params["touch"]["d2"].as<double>());
          ctrl.angular_stiffness = params["touch"]["ak"].as<double>();
          ctrl.angular_damping = params["touch"]["ad"].as<double>();

          probeState = TOUCH;
          std::cout << "### STARTING TOUCH PHASE" << std::endl;
        }
        break;
      case TOUCH:
        if (abs(ftWrenchInRobotFilt.get_force().z()) > params["touch"]["touch_force"].as<double>()) {
          std::cout << "Surface detected at position " << eeInRobot.get_position().transpose() << std::endl;

          jsonLogger.addTime();
          jsonLogger.addBody(logger::RAW, eeInRobot);
          jsonLogger.addField(logger::MessageType::CONTROL, "touch", true);
          jsonLogger.write();

          ++x_index;
          if (x_index >= params["grid"]["resolution"].as<double>()) {
            x_index = 0;
            ++y_index;
            if (y_index >= params["grid"]["resolution"].as<double>()) {
              y_index = 0;

              probeState = FINISH;
              std::cout << "### FINSHING" << std::endl;
              break;
            }
          }

          gains[2] = gains[1];
          pointDS.set_gain(gains);
          zVelocity = 0;

          // reset the controller to default damping values
          ctrl.set_linear_damping(params["default"]["d1"].as<double>(), params["default"]["d2"].as<double>());
          ctrl.angular_stiffness = params["default"]["ak"].as<double>();
          ctrl.angular_damping = params["default"]["ad"].as<double>();

          probeState = APPROACH;
          std::cout << "### STARTING TOUCH PHASE" << std::endl;
        }
        break;
      case FINISH:
        return 0;
    }

    CartesianTwist twistInTask = pointDS.evaluate(eeInTask);

    twistInTask.set_linear_velocity(twistInTask.get_linear_velocity() + Eigen::Vector3d(0, 0, zVelocity));
    auto commandTwistInRobot = CartesianTwist(taskInRobot * twistInTask);

    commandTwistInRobot.clamp(params["default"]["max_linear_velocity"].as<double>(),
          params["default"]["max_angular_velocity"].as<double>());

    CartesianWrench commandWrenchInRobot = ctrl.compute_command(commandTwistInRobot, eeInRobot);
    commandWrenchInRobot.clamp(params["default"]["max_control_force"].as<double>(),
                 params["default"]["max_control_torque"].as<double>());

    frankalwi::utils::fromJointTorque(jacobian.transpose() * commandWrenchInRobot, command);
    franka.send(command);

    jsonLogger.addTime();
    jsonLogger.addBody(logger::RAW, eeInRobot);
    jsonLogger.addBody(logger::RAW, taskInRobot);
    jsonLogger.addBody(logger::RAW, ftWrenchInRobot);

    jsonLogger.addBody(logger::FILTERED, eeInRobot);
    jsonLogger.addBody(logger::FILTERED, ftWrenchInRobotFilt);

    jsonLogger.addCommand(commandTwistInRobot, commandWrenchInRobot);

    jsonLogger.write();

    // frequency
    ++iterations;
    if (iterations > 1000) {
      std::chrono::duration<double> delta = std::chrono::system_clock::now() - frequencyTimer;
      std::cout << double(1000) / delta.count() << " Hz" << std::endl;
      frequencyTimer = std::chrono::system_clock::now();
      iterations = 0;
    }
  }
}