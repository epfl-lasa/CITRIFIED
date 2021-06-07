#include "controllers/IncisionTrialSystem.h"

#include <fstream>

using namespace state_representation;

IncisionTrialSystem::IncisionTrialSystem(const std::string& configFile) :
    ringDS(CartesianPose::Identity("center", "task")),
    ctrl(1, 1, 1, 1) {

  params = YAML::LoadFile(configFile);

  std::ifstream filestream(configFile);
  if (filestream.is_open()) {
    std::stringstream buf;
    buf << filestream.rdbuf();
    yamlContent = buf.str();
  }

  // Configure linear dynamical system
  CartesianPose center("center", "task");
  center.set_position(params["attractor"]["position"]["x"].as<double>(),
                      params["attractor"]["position"]["y"].as<double>(),
                      params["attractor"]["position"]["z"].as<double>());
  Eigen::Quaterniond orientation(0, 0, 1, 0);
  orientation = Eigen::AngleAxis<double>(params["attractor"]["pitch_angle"].as<double>() * M_PI / 180.0,
                                         -Eigen::Vector3d::UnitY()) * orientation;
  orientation =
      Eigen::AngleAxis<double>(params["attractor"]["yaw_angle"].as<double>() * M_PI / 180.0, Eigen::Vector3d::UnitZ())
          * orientation;
  center.set_orientation(orientation);

  auto linearGain = params["attractor"]["gains"]["linear"].as<double>();
  auto angularGain = params["attractor"]["gains"]["angular"].as<double>();
  DSgains = {linearGain, linearGain, linearGain, angularGain, angularGain, angularGain};

  pointDS.set_attractor(center);
  pointDS.set_gain(DSgains);
  pointDS.set_base_frame(CartesianState::Identity("task", "task"));

  // Configure circular dynamical system
  center.set_orientation(Eigen::Quaterniond::Identity());
  ringDS.set_center(center);
  ringDS.set_radius(params["cut"]["radius"].as<double>());
  ringDS.set_width(params["cut"]["width"].as<double>());
  ringDS.set_speed(params["cut"]["speed"].as<double>());
  ringDS.set_field_strength(params["cut"]["field_strength"].as<double>());
  ringDS.set_normal_gain(params["cut"]["normal_gain"].as<double>());
  ringDS.set_angular_gain(params["cut"]["angular_gain"].as<double>());
  ringDS.set_base_frame(CartesianState::Identity("task", "task"));

  // default pose at circle 0 angle (when local Y = 0 and local X > 0)
  //   has the knife X axis pointing along the world Y axis, with Z pointing down
  orientation = Eigen::Quaterniond(0, 1, 1, 0).normalized();
  orientation = Eigen::AngleAxis<double>(params["attractor"]["pitch_angle"].as<double>() * M_PI / 180.0,
                                         -Eigen::Vector3d::UnitX()) * orientation;
  ringDS.set_rotation_offset(orientation);

  // Configure controller
  ctrl = controllers::impedance::CartesianTwistController(params["default"]["d1"].as<double>(),
                                                          params["default"]["d2"].as<double>(),
                                                          params["default"]["ak"].as<double>(),
                                                          params["default"]["ad"].as<double>());

  cut = !params["insertion_only"].as<bool>();
  trialName = params["trial_prefix"].as<std::string>();
  if (cut) {
    trialName += "cut_" + params["trial"].as<std::string>() + ".json";
  } else {
    trialName += "insertion_" + params["trial"].as<std::string>() + ".json";
  }

  esnFilename = params["esn"]["filename"].as<std::string>();
  esnBufferSize = params["esn"]["buffer_size"].as<int>();
  esnMinTimeBetweenPredictions = params["esn"]["min_time_between_predictions"].as<double>();

  permittedClasses = params["esn"]["permitted_classes"].as<std::vector<std::string>>();
}

CartesianTwist IncisionTrialSystem::getTwistCommand(const CartesianState& eeInTask,
                                                    const CartesianState& taskInRobot,
                                                    TrialState trialState) const {
  CartesianTwist twistInTask("ee", "task");
  if (trialState == CUT) {
    twistInTask = ringDS.evaluate(eeInTask);
  } else if (trialState != PAUSE) {
    twistInTask = pointDS.evaluate(eeInTask);
  }

  twistInTask.set_linear_velocity(twistInTask.get_linear_velocity() + Eigen::Vector3d(0, 0, zVelocity));
  auto twistInRobot = CartesianTwist(taskInRobot * twistInTask);

  twistInRobot.clamp(params["default"]["max_linear_velocity"].as<double>(),
                     params["default"]["max_angular_velocity"].as<double>());

  return twistInRobot;
}

CartesianWrench
IncisionTrialSystem::getWrenchCommand(const CartesianTwist& twistInRobot, const CartesianState& eeInRobot) {
  CartesianWrench wrench = ctrl.compute_command(twistInRobot, eeInRobot);
  wrench.clamp(params["default"]["max_control_force"].as<double>(),
               params["default"]["max_control_torque"].as<double>());
  return wrench;
}

void IncisionTrialSystem::setDSBaseFrame(const CartesianState& base) {
  pointDS.set_base_frame(base);
  ringDS.set_base_frame(base);
}

void IncisionTrialSystem::setTouchPhase() {
  // set the point attractor to ignore Z axis, and instead add constant Z velocity
  DSgains[2] = 0;
  pointDS.set_gain(DSgains);
  zVelocity = -params["touch"]["speed"].as<double>();

  // set the controller to specific touch phase gains
  ctrl.set_linear_gains(params["touch"]["d1"].as<double>(), params["touch"]["d2"].as<double>());
  ctrl.set_angular_gains(params["touch"]["ak"].as<double>(), params["touch"]["ad"].as<double>());
}

void IncisionTrialSystem::setInsertionPhase() {
  // set the point attractor to ignore Z axis, and instead add constant Z velocity
  DSgains[2] = 0;
  pointDS.set_gain(DSgains);
  zVelocity = -params["insertion"]["speed"].as<double>();

  // set the controller to specific insertion phase gains
  ctrl.set_linear_gains(params["insertion"]["d1"].as<double>(), params["insertion"]["d2"].as<double>());
  ctrl.set_angular_gains(params["insertion"]["ak"].as<double>(), params["insertion"]["ad"].as<double>());
}

void IncisionTrialSystem::setCutPhase(const CartesianPose& eeInTask) {
  zVelocity = 0;
  // set circle center from current pose (radial distance along negative Y of local knife orientation)
  std::cout << "Current position: " << eeInTask.get_position().transpose() << std::endl;
  Eigen::Vector3d center(0, -ringDS.get_radius(), 0);
  center = eeInTask.get_orientation().toRotationMatrix() * center + eeInTask.get_position();
  center.z() = eeInTask.get_position().z();
  std::cout << "Estimated circle center at " << center.transpose() << std::endl;

  auto centerpose = ringDS.get_center();
  centerpose.set_position(center);
  ringDS.set_center(centerpose);

  // set the controller to specific cut phase gains
  ctrl.set_linear_gains(params["cut"]["d1"].as<double>(), params["cut"]["d2"].as<double>());
  ctrl.set_angular_gains(params["cut"]["ak"].as<double>(), params["cut"]["ad"].as<double>());
}

void IncisionTrialSystem::setRetractionPhase(const CartesianPose& eeInTask, double offset) {
  // set the point attractor right above the current pose
  auto targetPose = eeInTask;
  auto targetPosition = targetPose.get_position();
  targetPosition.z() += offset;
  targetPose.set_position(targetPosition);

  // reset the point attractor DS gains to respect Z axis
  pointDS.set_attractor(targetPose);
  DSgains[2] = DSgains[1];
  pointDS.set_gain(DSgains);
  zVelocity = 0;

  // reset the controller to default damping values
  ctrl.set_linear_gains(params["default"]["d1"].as<double>(), params["default"]["d2"].as<double>());
  ctrl.set_angular_gains(params["default"]["ak"].as<double>(), params["default"]["ad"].as<double>());
}