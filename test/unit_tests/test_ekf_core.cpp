#include <gtest/gtest.h>
#include "../../include/core/slam_system.hpp"

TEST(EkfSlamSystemTest, Initialization)
{
  ekf_slam::EkfSlamSystem slam(0.01, 0.01, 0.01, 0.5, 0.1, 2.0, 0.8);
  Eigen::Vector3d pose = slam.getCurrentPose();
  EXPECT_NEAR(pose(0), 0.0, 1e-6);
  EXPECT_NEAR(pose(1), 0.0, 1e-6);
  EXPECT_NEAR(pose(2), 0.0, 1e-6);
}

TEST(EkfSlamSystemTest, PredictMotion)
{
  ekf_slam::EkfSlamSystem slam(0.01, 0.01, 0.01, 0.5, 0.1, 2.0, 0.8);

  // v = 1.0 m/s, w = 0.0 rad/s, dt = 1.0 s → 직선 전진
  slam.predict(1.0, 0.0, 1.0);

  Eigen::Vector3d pose = slam.getCurrentPose();
  EXPECT_NEAR(pose(0), 1.0, 1e-3); // x 위치
  EXPECT_NEAR(pose(1), 0.0, 1e-3); // y 위치
  EXPECT_NEAR(pose(2), 0.0, 1e-3); // heading 변화 없음
}

TEST(EkfSlamSystemTest, LandmarkUpdate)
{
  ekf_slam::EkfSlamSystem slam(0.01, 0.01, 0.01, 0.5, 0.1, 2.0, 0.8);

  // 관측된 landmark 하나 추가
  ekf_slam::laser::Observation obs;
  obs.range = 1.0;
  obs.bearing = 0.0;

  std::vector<ekf_slam::laser::Observation> observations = { obs };

  slam.update(observations);

  // 랜드마크가 추가되었는지 확인
  Eigen::Vector3d pose = slam.getCurrentPose();
  EXPECT_NEAR(pose(0), 0.0, 1e-6);
  EXPECT_NEAR(pose(1), 0.0, 1e-6);
  EXPECT_NEAR(pose(2), 0.0, 1e-6);

  // landmark가 존재해야 한다
  EXPECT_TRUE(slam.hasLandmark(0));  // 첫 번째 landmark ID는 0일 가능성 높음
}
