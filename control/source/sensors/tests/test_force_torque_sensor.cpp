#include "gtest/gtest.h"

#include "sensors/ForceTorqueSensor.h"

class TestFTSensor : public ::testing::Test {
public:
  TestFTSensor() {
    tool.mass = 0.08;
    tool.centerOfMass = Eigen::Vector3d(0, 0, 0.025);
    mockSensorPtr = std::make_unique<sensors::ForceTorqueSensor>("mock_sensor", "0.0.0.0", 100, tool, true);
  }
  std::unique_ptr<sensors::ForceTorqueSensor> mockSensorPtr;
  sensors::ToolSpec tool;
  double biasCounter = 50;
  double tol = 1e-3;
};

TEST_F(TestFTSensor, RealSensorConnectionRefused) {
  ASSERT_ANY_THROW(std::unique_ptr<sensors::ForceTorqueSensor>
                       realSensorPtr(new sensors::ForceTorqueSensor("real_sensor", "192.168.1.1", 100, tool)));
}

TEST_F(TestFTSensor, MockSensorConnectionRefused) {
  StateRepresentation::CartesianWrench wrench("mock_sensor");
  ASSERT_TRUE(mockSensorPtr->readRawData(wrench));
}

TEST_F(TestFTSensor, MockSensorBias) {
  Eigen::Matrix3d rotation(Eigen::Matrix3d::Identity());
  bool success = mockSensorPtr->computeBias(rotation, biasCounter);
  std::size_t count = 0;
  while (!success && count < 2 * biasCounter) {
    success = mockSensorPtr->computeBias(rotation, biasCounter);
    ++count;
  }
  EXPECT_TRUE(success);
  EXPECT_EQ(count, biasCounter - 1);

  StateRepresentation::CartesianWrench bias("mock_sensor");
  EXPECT_TRUE(mockSensorPtr->readBias(bias));
  EXPECT_NEAR(bias.get_force().x(), 0, tol);
  EXPECT_NEAR(bias.get_force().y(), 0, tol);
  EXPECT_NEAR(bias.get_force().z(), tool.mass * 9.81, tol);
  EXPECT_NEAR(bias.get_torque().norm(), 0, tol);

  StateRepresentation::CartesianWrench wrench("mock_sensor");
  EXPECT_TRUE(mockSensorPtr->readContactWrench(wrench, rotation));
  EXPECT_NEAR(wrench.get_wrench().norm(), 0, tol);
}

TEST_F(TestFTSensor, MockSensorResetBias) {
  Eigen::Matrix3d rotation(Eigen::Matrix3d::Identity());
  StateRepresentation::CartesianWrench bias("mock_sensor");

  mockSensorPtr->resetBias();
  EXPECT_FALSE(mockSensorPtr->computeBias(rotation, biasCounter));
  EXPECT_FALSE(mockSensorPtr->readBias(bias));
}

TEST_F(TestFTSensor, MockSensorBiasGeneral) {
  Eigen::Matrix3d rotation(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix());
  bool success = mockSensorPtr->computeBias(rotation, biasCounter);
  std::size_t count = 0;
  while (!success && count < 2 * biasCounter) {
    success = mockSensorPtr->computeBias(rotation, biasCounter);
    ++count;
  }
  EXPECT_TRUE(success);
  EXPECT_EQ(count, biasCounter - 1);

  StateRepresentation::CartesianWrench bias("mock_sensor");
  EXPECT_TRUE(mockSensorPtr->readBias(bias));
  std::cout << bias << std::endl;
  Eigen::Matrix<double, 6, 1> wrench;
  wrench.head<3>() = -tool.mass * rotation * Eigen::Vector3d::UnitZ() * 9.81;
  wrench.tail<3>() = wrench.head<3>().cross(tool.centerOfMass);
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(bias.get_wrench()(i), wrench(i), tol);
  }
}
