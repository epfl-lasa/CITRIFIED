#pragma once

#include <fstream>
#include <iostream>
#include <string>

#include <json.hpp>

#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>

using json = nlohmann::json;

namespace logger {

enum MessageType {
  METADATA,
  STATIC,
  RAW,
  FILTERED,
  CONTROL,
  MODEL,
  ESN
};

class JSONLogger {
public:
  explicit JSONLogger(std::string filename = "", const std::string& prefix = "/tmp/");

  void setPrecision(int precision = 4);
  bool write();
  void print();
  void clear();

  void addMetaData(const std::string& trialID, const std::string& details="");

  template<class T>
  bool addBody(MessageType type, const T& body);

  void addCommand(const state_representation::CartesianTwist& twist, const state_representation::CartesianWrench& wrench);

  static std::string mapType(MessageType type) ;

  json object;

private:
  template<class T>
  json createBody(const T& body);

  std::ofstream file_;
};

template<class T>
bool JSONLogger::addBody(MessageType type, const T& body) {
  const auto& strtype = mapType(type);
  switch (type) {
    case STATIC:
    case RAW:
    case FILTERED:
      break;
    default:
      std::cerr << "Tried to add body array to message type " << strtype << std::endl;
      return false;
  }

  if (!object.contains(strtype)) {
    object[strtype] = json::object();
  }
  if (!object[strtype].contains("bodies") || !object[strtype]["bodies"].is_array()) {
    object[strtype]["bodies"] = json::array();
  }

  object[strtype]["bodies"].push_back(createBody<T>(body));
  return true;
}

}