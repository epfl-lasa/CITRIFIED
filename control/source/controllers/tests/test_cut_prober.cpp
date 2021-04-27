#include <gtest/gtest.h>

#include "controllers/CutProber.h"

class TestCutProber : public ::testing::Test {
protected:
  TestCutProber() :
    CP(std::string(CONTROLLERS_TEST_FIXTURES) + "incision_trial_parameters.yaml"),
    pose(state_representation::CartesianPose::Identity("pose")),
    xy_samples(2, 10), z_samples(10) {
      set_samples();
  }

  void set_samples() {
    xy_samples << 0.0141, 0.0091, 0.0042, -0.0010, -0.0058, -0.0108, -0.0159, -0.0209, -0.0261, -0.0310,
      0.0302, 0.0300, 0.0299, 0.0298, 0.0297, 0.0295, 0.0294, 0.0294, 0.0292, 0.0289;
    z_samples << 0.0519, 0.0551, 0.0585, 0.0607, 0.0626, 0.0629, 0.0621, 0.0610, 0.0590, 0.0559;
  }

  Eigen::Matrix2Xd xy_samples;
  Eigen::VectorXd z_samples;
  state_representation::CartesianPose pose;
  CutProber CP;
};

TEST_F(TestCutProber, add) {
  for (std::size_t sample = 0; sample < xy_samples.cols(); ++sample) {
    pose.set_position(xy_samples(0, sample), xy_samples(1, sample), z_samples(sample));
    CP.addPoint(pose);
  }

  pose.set_position(0, 0, 0);
  EXPECT_NEAR(CP.estimateHeightInTask(pose), 0.0596, 1e-3);

  pose.set_position(0, 0.03, 0);
  EXPECT_NEAR(CP.estimateHeightInTask(pose), 0.0603, 1e-3);
}