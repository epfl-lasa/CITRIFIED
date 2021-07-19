#include <gtest/gtest.h>
#include <iostream>

#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>

#include "logger/JSONLogger.h"


const static std::string logfileName = "test_json_logger_output.json";

class JSONLoggerTest : public ::testing::Test {
protected:
  JSONLoggerTest() : logger_(logfileName, "") {
  }

  void TearDown() override {
    std::remove(logfileName.c_str());
  }

  logger::JSONLogger logger_;
};

TEST_F(JSONLoggerTest, MetaData) {
  logger_.addMetaData("trialname", "some description");

  ASSERT_TRUE(logger_.object.contains("metadata"));
  auto md = logger_.object["metadata"];

  ASSERT_TRUE(md.contains("datetime"));
  EXPECT_GT(md["datetime"].get<std::string>().size(), 0);

  ASSERT_TRUE(md.contains("version"));
  EXPECT_GT(md["version"].get<std::string>().size(), 0);

  ASSERT_TRUE(md.contains("trial"));
  EXPECT_STREQ(md["trial"].get<std::string>().c_str(), "trialname");

  ASSERT_TRUE(md.contains("details"));
  EXPECT_STREQ(md["details"].get<std::string>().c_str(), "some description");
}

TEST_F(JSONLoggerTest, AddPose) {
  auto body = state_representation::CartesianPose::Random("A", "B");
  logger_.addBody(logger::RAW, body);
  json contents;
  try {
    contents = logger_.object.at(logger::JSONLogger::mapType(logger::RAW))["bodies"];
    ASSERT_TRUE(contents.is_array());
    EXPECT_EQ(contents.size(), 1);
    auto b = contents[0];
    EXPECT_STREQ(b["name"].get<std::string>().c_str(), "A");
    EXPECT_STREQ(b["frame"].get<std::string>().c_str(), "B");
    EXPECT_TRUE(b["pose"]["position"].is_array());
    EXPECT_EQ(b["pose"]["position"].size(), 3);
    ASSERT_TRUE(b["pose"]["orientation"].is_array());
    EXPECT_EQ(b["pose"]["orientation"].size(), 4);
  } catch (...) {
    FAIL();
  }
}

TEST_F(JSONLoggerTest, AddTwist) {
  auto body = state_representation::CartesianTwist::Random("A", "B");
  logger_.addBody(logger::RAW, body);

  json contents;
  try {
    contents = logger_.object.at(logger::JSONLogger::mapType(logger::RAW))["bodies"];
    ASSERT_TRUE(contents.is_array());
    EXPECT_EQ(contents.size(), 1);
    auto b = contents[0];
    EXPECT_STREQ(b["name"].get<std::string>().c_str(), "A");
    EXPECT_STREQ(b["frame"].get<std::string>().c_str(), "B");
    ASSERT_TRUE(b["twist"]["linear"].is_array());
    EXPECT_EQ(b["twist"]["linear"].size(), 3);
    ASSERT_TRUE(b["twist"]["angular"].is_array());
    EXPECT_EQ(b["twist"]["angular"].size(), 3);
  } catch (...) {
    FAIL();
  }
}

TEST_F(JSONLoggerTest, AddWrench) {
  auto body = state_representation::CartesianWrench::Random("A", "B");
  logger_.addBody(logger::RAW, body);

  json contents;
  try {
    contents = logger_.object.at(logger::JSONLogger::mapType(logger::RAW))["bodies"];
    ASSERT_TRUE(contents.is_array());
    EXPECT_EQ(contents.size(), 1);
    auto b = contents[0];
    EXPECT_STREQ(b["name"].get<std::string>().c_str(), "A");
    EXPECT_STREQ(b["frame"].get<std::string>().c_str(), "B");
    ASSERT_TRUE(b["wrench"]["force"].is_array());
    EXPECT_EQ(b["wrench"]["force"].size(), 3);
    ASSERT_TRUE(b["wrench"]["torque"].is_array());
    EXPECT_EQ(b["wrench"]["torque"].size(), 3);
  } catch (...) {
    FAIL();
  }
}

TEST_F(JSONLoggerTest, AddState) {
  auto body = state_representation::CartesianState::Random("A", "B");
  logger_.addBody(logger::RAW, body);

  json contents;
  try {
    contents = logger_.object.at(logger::JSONLogger::mapType(logger::RAW))["bodies"];
    ASSERT_TRUE(contents.is_array());
    EXPECT_EQ(contents.size(), 1);
    auto b = contents[0];
    EXPECT_STREQ(b["name"].get<std::string>().c_str(), "A");
    EXPECT_STREQ(b["frame"].get<std::string>().c_str(), "B");
    ASSERT_TRUE(b["pose"]["position"].is_array());
    EXPECT_EQ(b["pose"]["position"].size(), 3);
    ASSERT_TRUE(b["pose"]["orientation"].is_array());
    EXPECT_EQ(b["pose"]["orientation"].size(), 4);
    ASSERT_TRUE(b["twist"]["linear"].is_array());
    EXPECT_EQ(b["twist"]["linear"].size(), 3);
    ASSERT_TRUE(b["twist"]["angular"].is_array());
    EXPECT_EQ(b["twist"]["angular"].size(), 3);
    ASSERT_TRUE(b["wrench"]["force"].is_array());
    EXPECT_EQ(b["wrench"]["force"].size(), 3);
    ASSERT_TRUE(b["wrench"]["torque"].is_array());
    EXPECT_EQ(b["wrench"]["torque"].size(), 3);
  } catch (...) {
    FAIL();
  }
}

TEST_F(JSONLoggerTest, AddBodyTypeCheck) {
  auto pose = state_representation::CartesianPose::Random("A", "B");

  auto type = logger::JSONLogger::mapType(logger::STATIC);
  EXPECT_FALSE(logger_.object.contains(type));
  EXPECT_TRUE(logger_.addBody(logger::STATIC, pose));
  EXPECT_TRUE(logger_.object.contains(type));

  type = logger::JSONLogger::mapType(logger::RAW);
  EXPECT_FALSE(logger_.object.contains(type));
  EXPECT_TRUE(logger_.addBody(logger::RAW, pose));
  EXPECT_TRUE(logger_.object.contains(type));

  type = logger::JSONLogger::mapType(logger::FILTERED);
  EXPECT_FALSE(logger_.object.contains(type));
  EXPECT_TRUE(logger_.addBody(logger::FILTERED, pose));
  EXPECT_TRUE(logger_.object.contains(type));

  type = logger::JSONLogger::mapType(logger::CONTROL);
  EXPECT_FALSE(logger_.object.contains(type));
  EXPECT_FALSE(logger_.addBody(logger::CONTROL, pose));
  EXPECT_FALSE(logger_.object.contains(type));
}

TEST_F(JSONLoggerTest, AddCommand) {
  auto commandTwist = state_representation::CartesianTwist::Random("A", "B");
  auto commandWrench = state_representation::CartesianWrench::Random("A", "B");

  logger_.addCommand(commandTwist, commandWrench);
  ASSERT_TRUE(logger_.object.contains("control"));
  ASSERT_TRUE(logger_.object["control"].contains("command"));
  auto command = logger_.object["control"]["command"];
  EXPECT_STREQ(command["name"].get<std::string>().c_str(), "A");
  EXPECT_STREQ(command["frame"].get<std::string>().c_str(), "B");
  ASSERT_TRUE(command["twist"]["linear"].is_array());
  EXPECT_EQ(command["twist"]["linear"].size(), 3);
  ASSERT_TRUE(command["twist"]["angular"].is_array());
  EXPECT_EQ(command["twist"]["angular"].size(), 3);
  ASSERT_TRUE(command["wrench"]["force"].is_array());
  EXPECT_EQ(command["wrench"]["force"].size(), 3);
  ASSERT_TRUE(command["wrench"]["torque"].is_array());
  EXPECT_EQ(command["wrench"]["torque"].size(), 3);
}

TEST_F(JSONLoggerTest, AddField) {
  logger_.addField(logger::CONTROL, "phase", "test");
  ASSERT_TRUE(logger_.object.contains("control"));
  ASSERT_TRUE(logger_.object["control"].contains("phase"));
  ASSERT_TRUE(logger_.object["control"]["phase"].is_string());
  EXPECT_STREQ(logger_.object["control"]["phase"].get<std::string>().c_str(), "test");
}

TEST_F(JSONLoggerTest, AddTime) {
  EXPECT_FALSE(logger_.object.contains("time"));
  logger_.addTime();
  ASSERT_TRUE(logger_.object.contains("time"));
  ASSERT_TRUE(logger_.object["time"].is_number());
  EXPECT_GT(logger_.object["time"].get<double>(), 0);
}

TEST_F(JSONLoggerTest, Clear) {
  EXPECT_TRUE(logger_.object.empty());
  logger_.addMetaData("A");
  EXPECT_FALSE(logger_.object.empty());
  logger_.clear();
  EXPECT_TRUE(logger_.object.empty());
}