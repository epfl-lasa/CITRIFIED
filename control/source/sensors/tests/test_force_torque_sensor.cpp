#include "gtest/gtest.h"

#include "sensors/ForceTorqueSensor.h"

class TestFTSensor : public ::testing::Test {
public:
  TestFTSensor() {
    tool.mass = 0.08;
    tool.centerOfMass = Eigen::Vector3d(0, 0, 0.025);
    simSensorPtr = std::make_unique<sensors::ForceTorqueSensor>("sim_sensor", "0.0.0.0", 100, tool, true);
  }
  std::unique_ptr<sensors::ForceTorqueSensor> simSensorPtr;
  std::unique_ptr<sensors::ForceTorqueSensor> realSensorPtr;
  sensors::ToolSpec tool;
};

TEST_F(TestFTSensor, RealSensorConnectionRefused) {
  EXPECT_ANY_THROW(
      realSensorPtr = std::make_unique<sensors::ForceTorqueSensor>("real_sensor", "192.168.1.1", 100, tool, false));
}
