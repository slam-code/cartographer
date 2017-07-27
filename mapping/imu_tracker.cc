
#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/*

imu_gravity_time_constant = 10.,
num_odometry_states = 1000,
构造函数初始化
*/
ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),// 1796年
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(Eigen::Vector3d::UnitZ()), //0,0,1，只有z轴有重力加速度。
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

/*
从time_增加到time。
dt=t-t_;
ds=v*dt

*/
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);

  //方向角
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t)); //ds=v*dt

  orientation_ = (orientation_ * rotation).normalized();//更新角度
  gravity_vector_ = rotation.inverse() * gravity_vector_;//根据方向角，更新加速度

  time_ = time;
}

/*
更新imu测量得到的加速度。
参数：vector3d，测量值
1),dt=t_-t;
2),alpha=1-e^(-dt/g);
3),gravity_vector_=(1-alpha)*gv_+alpha*imu_line;
4),更新orientation_


exponential,指数.
为何是指数变化：
*/
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      gravity_vector_, orientation_.inverse() * Eigen::Vector3d::UnitZ());

  orientation_ = (orientation_ * rotation).normalized();//更新方向角

  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

/*
更新imu测量得到的角速度
*/
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
