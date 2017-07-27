
#include "cartographer/kalman_filter/pose_tracker.h"

#include <random>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace kalman_filter {
namespace {

constexpr double kOdometerVariance = 1e-12;

using transform::IsNearly;
using transform::Rigid3d;
using ::testing::Not;

class PoseTrackerTest : public ::testing::Test {
 protected:
  PoseTrackerTest() {
    auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
            orientation_model_variance = 1e-8,
            position_model_variance = 1e-8,
            velocity_model_variance = 1e-8,
            imu_gravity_time_constant = 100.,
            imu_gravity_variance = 1e-9,
            num_odometry_states = 1,
        }
        )text");
    const proto::PoseTrackerOptions options =
        CreatePoseTrackerOptions(parameter_dictionary.get());
    pose_tracker_ =
        common::make_unique<PoseTracker>(options, common::FromUniversal(1000)); //创建卡尔曼滤波跟踪器
  }

  std::unique_ptr<PoseTracker> pose_tracker_;
};

TEST_F(PoseTrackerTest, SaveAndRestore) {
  std::vector<Rigid3d> poses(3);
  std::vector<PoseCovariance> covariances(3);
  //0号，获取t=1500处的预测值
  pose_tracker_->GetPoseEstimateMeanAndCovariance(common::FromUniversal(1500),
                                                  &poses[0], &covariances[0]);

  //0号，添加t=2000的测量值
  pose_tracker_->AddImuLinearAccelerationObservation(
      common::FromUniversal(2000), Eigen::Vector3d(1, 1, 9));

  PoseTracker copy_of_pose_tracker = *pose_tracker_;//默认copy构造函数，1号

  const Eigen::Vector3d observation(2, 0, 8);
  //0号，添加t=3000的测量值
  pose_tracker_->AddImuLinearAccelerationObservation(
      common::FromUniversal(3000), observation);

  //0号，获取t=3500的预测值
  pose_tracker_->GetPoseEstimateMeanAndCovariance(common::FromUniversal(3500),
                                                  &poses[1], &covariances[1]);

  //1号，添加t=3000的测量值
  copy_of_pose_tracker.AddImuLinearAccelerationObservation(
      common::FromUniversal(3000), observation);
  //1号，获取t=3500的测量值
  copy_of_pose_tracker.GetPoseEstimateMeanAndCovariance(
      common::FromUniversal(3500), &poses[2], &covariances[2]);

  EXPECT_THAT(poses[0], Not(IsNearly(poses[1], 1e-6)));
  EXPECT_FALSE((covariances[0].array() == covariances[1].array()).all());

  //相同的测量，导致相同的结果 
  EXPECT_THAT(poses[1], IsNearly(poses[2], 1e-6));
  EXPECT_TRUE((covariances[1].array() == covariances[2].array()).all());
}

TEST_F(PoseTrackerTest, AddImuLinearAccelerationObservation) {
  auto time = common::FromUniversal(1000);

  for (int i = 0; i < 300; ++i) {
    time += std::chrono::seconds(5);
    //循环300次，添加测量值
    pose_tracker_->AddImuLinearAccelerationObservation(
        time, Eigen::Vector3d(0., 0., 10.));
  }

  {
    Rigid3d pose;
    PoseCovariance covariance;
    pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose, &covariance);
    const Eigen::Quaterniond actual = Eigen::Quaterniond(pose.rotation());
    const Eigen::Quaterniond expected = Eigen::Quaterniond::Identity();
    //expected.coeffs():0,0,0,1
    //预测值和真实值相等
    EXPECT_TRUE(actual.isApprox(expected, 1e-3)) << expected.coeffs() << " vs\n"
                                                 << actual.coeffs();
  }

  for (int i = 0; i < 300; ++i) {
    time += std::chrono::seconds(5);
    pose_tracker_->AddImuLinearAccelerationObservation(
        time, Eigen::Vector3d(0., 10., 0.));
  }

  time += std::chrono::milliseconds(5);

  Rigid3d pose;
  PoseCovariance covariance;
  //获取预测值
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose, &covariance);
  const Eigen::Quaterniond actual = Eigen::Quaterniond(pose.rotation());
  const Eigen::Quaterniond expected = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX()));
  //预测值与真实值比较
  EXPECT_TRUE(actual.isApprox(expected, 1e-3)) << expected.coeffs() << " vs\n"
                                               << actual.coeffs();
}

TEST_F(PoseTrackerTest, AddImuAngularVelocityObservation) {
  auto time = common::FromUniversal(1000);

  for (int i = 0; i < 300; ++i) {
    time += std::chrono::milliseconds(5);
    pose_tracker_->AddImuAngularVelocityObservation(time,
                                                    Eigen::Vector3d::Zero());
  }

  {
    Rigid3d pose;
    PoseCovariance covariance;
    pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose, &covariance);
    const Eigen::Quaterniond actual = Eigen::Quaterniond(pose.rotation());
    const Eigen::Quaterniond expected = Eigen::Quaterniond::Identity();
    EXPECT_TRUE(actual.isApprox(expected, 1e-3)) << expected.coeffs() << " vs\n"
                                                 << actual.coeffs();
  }

  const double target_radians = M_PI / 2.;
  const double num_observations = 300.;
  const double angular_velocity = target_radians / (num_observations * 5e-3);
  for (int i = 0; i < num_observations; ++i) {
    time += std::chrono::milliseconds(5);
    pose_tracker_->AddImuAngularVelocityObservation(
        time, Eigen::Vector3d(angular_velocity, 0., 0.));
  }

  time += std::chrono::milliseconds(5);

  Rigid3d pose;
  PoseCovariance covariance;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose, &covariance);
  const Eigen::Quaterniond actual = Eigen::Quaterniond(pose.rotation());
  const Eigen::Quaterniond expected = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(actual.isApprox(expected, 1e-3)) << expected.coeffs() << " vs\n"
                                               << actual.coeffs();
}

TEST_F(PoseTrackerTest, AddPoseObservation) {
  auto time = common::FromUniversal(1000);

  for (int i = 0; i < 300; ++i) {
    time += std::chrono::milliseconds(5);
    pose_tracker_->AddPoseObservation(
        time, Rigid3d::Identity(),
        Eigen::Matrix<double, 6, 6>::Identity() * 1e-6);
  }

  {
    Rigid3d actual;
    PoseCovariance covariance;
    pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &actual, &covariance);
    EXPECT_THAT(actual, IsNearly(Rigid3d::Identity(), 1e-3));
  }

  const Rigid3d expected =
      Rigid3d::Translation(Eigen::Vector3d(1., 2., 3.)) *
      Rigid3d::Rotation(Eigen::AngleAxisd(
          M_PI / 2., Eigen::Vector3d(0., 0., 3.).normalized()));

  for (int i = 0; i < 300; ++i) {
    time += std::chrono::milliseconds(15);
    pose_tracker_->AddPoseObservation(
        time, expected, Eigen::Matrix<double, 6, 6>::Identity() * 1e-9);
  }

  time += std::chrono::milliseconds(15);

  Rigid3d actual;
  PoseCovariance covariance;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &actual, &covariance);
  EXPECT_THAT(actual, IsNearly(expected, 1e-3));
}

TEST_F(PoseTrackerTest, AddOdometerPoseObservation) {
  common::Time time = common::FromUniversal(0);

  std::vector<Rigid3d> odometer_track;
  odometer_track.push_back(Rigid3d::Identity());
  odometer_track.push_back(
      Rigid3d::Rotation(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())));
  odometer_track.push_back(
      Rigid3d::Translation(Eigen::Vector3d(0.2, 0., 0.)) *
      Rigid3d::Rotation(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())));
  odometer_track.push_back(
      Rigid3d::Translation(Eigen::Vector3d(0.3, 0.1, 0.)) *
      Rigid3d::Rotation(Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ())));
  odometer_track.push_back(
      Rigid3d::Translation(Eigen::Vector3d(0.2, 0.2, 0.1)) *
      Rigid3d::Rotation(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())));
  odometer_track.push_back(
      Rigid3d::Translation(Eigen::Vector3d(0.1, 0.2, 0.2)) *
      Rigid3d::Rotation(Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ())));
  odometer_track.push_back(Rigid3d::Translation(Eigen::Vector3d(0., 0.1, 0.2)));

  Rigid3d actual;
  PoseCovariance unused_covariance;
  for (const Rigid3d& pose : odometer_track) {
    time += std::chrono::seconds(1);
    pose_tracker_->AddOdometerPoseObservation(
        time, pose, kOdometerVariance * PoseCovariance::Identity());
    pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &actual,
                                                    &unused_covariance);
    EXPECT_THAT(actual, IsNearly(pose, 1e-2));
  }
  // Sanity check that the test has signal:
  EXPECT_THAT(actual, Not(IsNearly(odometer_track[0], 1e-2)));
}

}  // namespace
}  // namespace kalman_filter
}  // namespace cartographer
