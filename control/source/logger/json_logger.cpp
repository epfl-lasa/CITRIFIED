#include <ctime>
#include <iomanip>
#include "logger/json_logger.h"

#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

#define JSON_LOGGER_VERSION ("1.0")

using namespace state_representation;

namespace logger {
static const std::map<MessageType, std::string> messageMap {
    {METADATA, "metadata"},
    {STATIC, "static"},
    {RAW, "raw"},
    {FILTERED, "filtered"},
    {CONTROL, "control"},
    {MODEL, "model"},
    {ESN, "esn"}
};

std::string JSONLogger::mapType(MessageType type) {
  return messageMap.at(type);
}

JSONLogger::JSONLogger(std::string filename, const std::string& prefix) {
  if (filename.empty()) {
    auto t = std::time(nullptr);
    char datetime[30];
    std::strftime(datetime, 30, "%Y-%m-%d-%H-%M-%S", std::gmtime(&t));
    filename = std::string(datetime) + ".json";
  }

  file_.open(prefix + filename, std::ofstream::out | std::ofstream::trunc);
  setPrecision(4);
}

void JSONLogger::setPrecision(int precision) {
  file_ << std::fixed << std::setprecision(precision);
}

template<>
json JSONLogger::createBody<CartesianPose>(const CartesianPose& body) {
  auto bodyObject = json::object();
  bodyObject["name"] = body.get_name();
  bodyObject["pose"]["position"] = {
      body.get_position().x(),
      body.get_position().y(),
      body.get_position().z()
  };
  bodyObject["pose"]["orientation"] = {
      body.get_orientation().w(),
      body.get_orientation().x(),
      body.get_orientation().y(),
      body.get_orientation().z()
  };
  bodyObject["frame"] = body.get_reference_frame();
  return bodyObject;
}

template<>
json JSONLogger::createBody(const CartesianTwist& body) {
  auto bodyObject = json::object();
  bodyObject["name"] = body.get_name();
  bodyObject["twist"]["linear"] = {
      body.get_linear_velocity().x(),
      body.get_linear_velocity().y(),
      body.get_linear_velocity().z()
  };
  bodyObject["twist"]["angular"] = {
      body.get_angular_velocity().x(),
      body.get_angular_velocity().y(),
      body.get_angular_velocity().z()
  };
  bodyObject["frame"] = body.get_reference_frame();
  return bodyObject;
}

template<>
json JSONLogger::createBody(const CartesianWrench& body) {
  auto bodyObject = json::object();
  bodyObject["name"] = body.get_name();
  bodyObject["wrench"]["force"] = {
      body.get_force().x(),
      body.get_force().y(),
      body.get_force().z()
  };
  bodyObject["wrench"]["torque"] = {
      body.get_torque().x(),
      body.get_torque().y(),
      body.get_torque().z()
  };
  bodyObject["frame"] = body.get_reference_frame();
  return bodyObject;
}

template<>
json JSONLogger::createBody(const CartesianState& body) {
  auto bodyObject = createBody<CartesianPose>(body);
  bodyObject["twist"] = createBody<CartesianTwist>(body)["twist"];
  bodyObject["wrench"] = createBody<CartesianWrench>(body)["wrench"];

  return bodyObject;
}


bool JSONLogger::write() {
  if (file_.is_open()) {
    file_ << object << std::endl;
    clear();
    return true;
  }
  return false;
}

void JSONLogger::print() {
  std::cout << object << std::endl;
}

void JSONLogger::clear() {
  object = json::object();
}

void JSONLogger::addMetaData(const std::string& trialID, const std::string& details) {
  auto t = std::time(nullptr);
  char datetime[30];
  std::strftime(datetime, 30, "%FT%TZ", std::gmtime(&t));
  object["metadata"] = {
      {"version", std::string(JSON_LOGGER_VERSION)},
      {"datetime", std::string(datetime)},
      {"trial", trialID},
      {"details", details}
  };
}

void JSONLogger::addCommand(const state_representation::CartesianTwist& twist,
                            const state_representation::CartesianWrench& wrench) {
  auto command = createBody(twist);
  command["wrench"] = createBody(wrench)["wrench"];
  object["control"]["command"] = command;
}

}