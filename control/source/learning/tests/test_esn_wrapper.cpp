#include "learning/ESNWrapper.h"

#include <stdexcept>
#include <gtest/gtest.h>

using namespace learning;

class ESNWrapperTest : public testing::Test {
protected:
  void SetUp() override {
    path = std::string(LEARNING_TEST_FIXTURES) + "test_esn.yaml";
  }

  std::string path;

  double tol = 1e-3;
};

TEST_F(ESNWrapperTest, TestPrediction) {
  int bufferSize = 10;
  auto esn = ESNWrapper(path, bufferSize);
  esn.setDerivativeCalculationIndices({2, 3});

  Eigen::Vector4d sample;
  double dt = 0.01;
  for (int i = 0; i < bufferSize; ++i) {
    sample << i, 2 * i, sin(i * dt), cos(i * dt);
    esn.addSample(i * dt, sample);

    EXPECT_FALSE(esn.getLastPrediction());
    if (i < bufferSize - 1) {
      EXPECT_FALSE(esn.classify());
    }
  }

  EXPECT_TRUE(esn.classify());
  Eigen::MatrixXd time, data;
  auto prediction = esn.getLastPrediction(time, data);
  ASSERT_TRUE(prediction);

  EXPECT_EQ(time.size(), bufferSize);
  EXPECT_EQ(data.rows(), bufferSize);

  EXPECT_NEAR(data(0, 4), data(1, 4), tol);
  EXPECT_NEAR(data(0, 5), data(1, 5), tol);
  for (int i = 1; i < bufferSize - 1; ++i) {
    EXPECT_NEAR(time(i), i * dt, tol);
    EXPECT_NEAR(data(i, 4), cos(i * dt), tol);
    EXPECT_NEAR(data(i, 5), -sin(i * dt), tol);
  }
  EXPECT_NEAR(data(bufferSize - 1, 4), data(bufferSize - 2, 4), tol);
  EXPECT_NEAR(data(bufferSize - 1, 5), data(bufferSize - 2, 5), tol);

  //trying to retrieve the prediction again should fail
  EXPECT_FALSE(esn.getLastPrediction());
}