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
  sensors::ToolSpec tool;
  double tol = 1e-3;
};

TEST_F(TestFTSensor, RealSensorConnectionRefused) {
  ASSERT_ANY_THROW(std::unique_ptr<sensors::ForceTorqueSensor>
                       realSensorPtr(new sensors::ForceTorqueSensor("real_sensor", "192.168.1.1", 100, tool, false)));
}

TEST_F(TestFTSensor, SimSensorConnectionRefused) {
  StateRepresentation::CartesianWrench wrench("sim_sensor");
  ASSERT_TRUE(simSensorPtr->readRawData(wrench));
}

TEST_F(TestFTSensor, SimSensorBias) {
  Eigen::Matrix3d rotation(Eigen::Matrix3d::Identity());
  bool success = simSensorPtr->computeBias(rotation, 50);
  std::size_t count = 0;
  while (!success) {
    success = simSensorPtr->computeBias(rotation, 50);
    ++count;
  }
  EXPECT_TRUE(success);
  EXPECT_EQ(count, 50 - 1);

  StateRepresentation::CartesianWrench bias("sim_sensor");
  EXPECT_TRUE(simSensorPtr->readBias(bias));
  EXPECT_NEAR(bias.get_force().x(), 0, tol);
  EXPECT_NEAR(bias.get_force().y(), 0, tol);
  EXPECT_NEAR(bias.get_force().z(), tool.mass * 9.81, tol);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(bias.get_torque()(i), 0, tol);
  }

  StateRepresentation::CartesianWrench wrench("sim_sensor");
  EXPECT_TRUE(simSensorPtr->readContactWrench(wrench, rotation));
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(wrench.get_wrench()(i), 0, tol);
  }
}

TEST_F(TestFTSensor, SimSensorResetBias) {
  Eigen::Matrix3d rotation(Eigen::Matrix3d::Identity());
  StateRepresentation::CartesianWrench bias("sim_sensor");

  simSensorPtr->resetBias();
  EXPECT_FALSE(simSensorPtr->computeBias(rotation, 50));
  EXPECT_FALSE(simSensorPtr->readBias(bias));
}

TEST_F(TestFTSensor, SimSensorBiasGeneral) {
  Eigen::Matrix3d rotation(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix());
  bool success = simSensorPtr->computeBias(rotation, 50);
  std::size_t count = 0;
  while (!success) {
    success = simSensorPtr->computeBias(rotation, 50);
    ++count;
  }
  EXPECT_TRUE(success);
  EXPECT_EQ(count, 50 - 1);

  StateRepresentation::CartesianWrench bias("sim_sensor");
  EXPECT_TRUE(simSensorPtr->readBias(bias));
  std::cout << bias << std::endl;
  Eigen::Matrix<double, 6, 1> wrench;
  wrench.head<3>() = -tool.mass * rotation * Eigen::Vector3d::UnitZ() * 9.81;
  wrench.tail<3>() = wrench.head<3>().cross(tool.centerOfMass);
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(bias.get_wrench()(i), wrench(i), tol);
  }
}
