
#ifndef CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_
#define CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_

#include <deque>
#include <memory>

#include "Eigen/Cholesky"
#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/gaussian_distribution.h"
#include "cartographer/kalman_filter/proto/pose_tracker_options.pb.h"
#include "cartographer/kalman_filter/unscented_kalman_filter.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/odometry_state_tracker.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace kalman_filter {

typedef Eigen::Matrix3d Pose2DCovariance; //3*3矩阵
typedef Eigen::Matrix<double, 6, 6> PoseCovariance;// 6*6 矩阵

struct PoseAndCovariance {
  transform::Rigid3d pose;
  PoseCovariance covariance; //6*6
};

PoseAndCovariance operator*(const transform::Rigid3d& transform,
                            const PoseAndCovariance& pose_and_covariance);

PoseCovariance BuildPoseCovariance(double translational_variance,
                                   double rotational_variance);

proto::PoseTrackerOptions CreatePoseTrackerOptions(
    common::LuaParameterDictionary* parameter_dictionary);

/*
基于卡尔曼滤波的3维位置和方向估计

*/
// A Kalman filter for a 3D state of position and orientation.
// Includes functions to update from IMU and range data matches.
class PoseTracker {
 public:
  enum {
    kMapPositionX = 0,//位置信息{X,Y,Z}
    kMapPositionY,
    kMapPositionZ,
    kMapOrientationX,//方向信息,3
    kMapOrientationY,
    kMapOrientationZ,
    kMapVelocityX,   //速度信息,6
    kMapVelocityY,
    kMapVelocityZ,
    kDimension  //9, We terminate loops with this. 只追踪9个维度
  };

  using KalmanFilter = UnscentedKalmanFilter<double, kDimension>;//9维的卡尔曼滤波
  using State = KalmanFilter::StateType;                         //N*1矩阵
  using StateCovariance = Eigen::Matrix<double, kDimension, kDimension>;//9*9
  using Distribution = GaussianDistribution<double, kDimension>;
   //参数类型的double,9*1的矩阵

  // Create a new Kalman filter starting at the origin with a standard normal
  // distribution at 'time'.
  //在给定的time时刻初始化卡尔曼滤波参数
  PoseTracker(const proto::PoseTrackerOptions& options, common::Time time);
  virtual ~PoseTracker();

  // Sets 'pose' and 'covariance' to the mean and covariance of the belief at
  // 'time' which must be >= to the current time. Must not be nullptr.
  //通过指针获取pose的旋转参数和covariance方差
  void GetPoseEstimateMeanAndCovariance(common::Time time,
                                        transform::Rigid3d* pose,
                                        PoseCovariance* covariance);

  // Updates from an IMU reading (in the IMU frame).
  //根据imu观测值更新
  void AddImuLinearAccelerationObservation(
      common::Time time, const Eigen::Vector3d& imu_linear_acceleration);
  void AddImuAngularVelocityObservation(
      common::Time time, const Eigen::Vector3d& imu_angular_velocity);

  // Updates from a pose estimate in the map frame.
  //根据map-frame的位姿估计更新
  void AddPoseObservation(common::Time time, const transform::Rigid3d& pose,
                          const PoseCovariance& covariance);

  // Updates from a pose estimate in the odometer's map-like frame.
  //根据里程计的map-like frame位姿估计更新
  void AddOdometerPoseObservation(common::Time time,
                                  const transform::Rigid3d& pose,
                                  const PoseCovariance& covariance);

  common::Time time() const { return time_; } //最新有效时间

  // Returns the belief at the 'time' which must be >= to the current time.
  Distribution GetBelief(common::Time time); //未来某一时刻的状态估计值

  const mapping::OdometryStateTracker::OdometryStates& odometry_states() const;

//imu的重力方向
  Eigen::Quaterniond gravity_orientation() const {
    return imu_tracker_.orientation();
  }

 private:
  //返回初始状态的状态变量的高斯分布
  // Returns the distribution required to initialize the KalmanFilter.
  static Distribution KalmanFilterInit();

  //建立零均值噪声模型
  // Build a model noise distribution (zero mean) for the given time delta.
  const Distribution BuildModelNoise(double delta_t) const;

  // Predict the state forward in time. This is a no-op if 'time' has not moved
  // forward.
  //根据当前状态预测time时刻的状态
  void Predict(common::Time time);

  // Computes a pose combining the given 'state' with the 'imu_tracker_'
  // orientation.
  //结合 imu_tracker_和state,计算位姿pose的旋转变换。
  transform::Rigid3d RigidFromState(const PoseTracker::State& state);

  const proto::PoseTrackerOptions options_; //用于位姿估计的传感器特性
  common::Time time_;                       //测量时间
  KalmanFilter kalman_filter_;              //卡尔曼滤波
  mapping::ImuTracker imu_tracker_;         //imu跟踪
  mapping::OdometryStateTracker odometry_state_tracker_;//里程计跟踪
};

}  // namespace kalman_filter
}  // namespace cartographer

#endif  // CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_
