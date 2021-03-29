#pragma once

#include <chrono>
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

  bool write();
  void print();
  void clear();

  void addTime();
  double getTime() const;

  void addMetaData(const std::string& trialID, const std::string& details="");

  template<class T>
  void addField(MessageType type, const std::string& field, const T& value);

  template<class T>
  void addSubfield(MessageType type, const std::string& field, const std::string& subfield, const T& value);

  template<class T>
  bool addBody(MessageType type, const T& body);

  void addCommand(const state_representation::CartesianTwist& twist, const state_representation::CartesianWrench& wrench);

  static std::string mapType(MessageType type) ;

  json object;

private:
  template<class T>
  json createBody(const T& body);

  std::chrono::steady_clock::time_point startTime_;
  std::ofstream file_;
};

template<class T>
void JSONLogger::addField(MessageType type, const std::string& field, const T& value) {
  object[mapType(type)][field] = value;
}

template<class T>
void JSONLogger::addSubfield(MessageType type, const std::string& field, const std::string& subfield, const T& value) {
  object[mapType(type)][field][subfield] = value;
}

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