
#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {


/*
ImuTracker
作用：使用来自imu的角速度+加速度用于跟踪pose的orientation


构造函数指定重力加速度g和最新测量时间。
数据成员包含
1,加速度g
2,测量时间
3,方向,四元数
4,三个加速度3d
5,imu角速度

*/
// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, 
// roll/pitch does not drift,though yaw does.
//x,y轴不会drift漂移。z轴可能会产生漂移。(有累计误差)
class ImuTracker {
 public:
  ImuTracker(double imu_gravity_time_constant, common::Time time);


  // Advances to the given 'time' and updates the orientation to reflect this.
  // 系统时间增加t，更新方向角
  void Advance(common::Time time);

  // Updates from an IMU reading (in the IMU frame).
  void AddImuLinearAccelerationObservation(   //加速度
      const Eigen::Vector3d& imu_linear_acceleration);

  void AddImuAngularVelocityObservation(      //角速度
      const Eigen::Vector3d& imu_angular_velocity);  

  // Query the current orientation estimate.返回目前估计pose的方向角。
  Eigen::Quaterniond orientation() const { return orientation_; }

 private:
  const double imu_gravity_time_constant_;    // g，10.0
  common::Time time_;                         //最新一次测量时间
  common::Time last_linear_acceleration_time_;//加速度测量时间
  Eigen::Quaterniond orientation_;            //pose的方向角
  Eigen::Vector3d gravity_vector_;            //加速度测量的方向
  Eigen::Vector3d imu_angular_velocity_;      //角速度
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
/*
预备知识：
惯性测量单元（英文：Inertial measurement unit，简称IMU）是测量物体三轴姿态角(或角速率)以及加速度的装置。
陀螺仪及加速度计是IMU的主要元件，其精度直接影响到惯性系统的精度。在实际工作中，由于不可避免的各种干扰因素，而导致陀螺仪及加速度计产生误差，从初始对准开始，其导航误差就随时间而增长，尤其是位置误差，这是惯导系统的主要缺点。所以需要利用外部信息进行辅助，实现组合导航，使其有效地减小误差随时间积累的问题。为了提高可靠性，还可以为每个轴配备更多的传感器。一般而言IMU要安装在被测物体的重心上。
一般情况，一个IMU包含了三个单轴的加速度计和三个单轴的陀螺仪，加速度计检测物体在载体坐标系统独立三轴的加速度信号，而陀螺仪检测载体相对于导航坐标系的角速度信号，测量物体在三维空间中的角速度和加速度，并以此解算出物体的姿态。在导航中有着很重要的应用价值。
IMU大多用在需要进行运动控制的设备，如汽车和机器人上。也被用在需要用姿态进行精密位移推算的场合，如潜艇、飞机、导弹和航天器的惯性导航设备等。

利用三轴地磁解耦和三轴加速度计，受外力加速度影响很大，在运动/振动等环境中，输出方向角误差较大,此外地磁传感器有缺点，它的绝对参照物是地磁场的磁力线,地磁的特点是使用范围大，但强度较低，约零点几高斯，非常容易受到其它磁体的干扰， 如果融合了Z轴陀螺仪的瞬时角度，就可以使系统数据更加稳定。加速度测量的是重力方向，在无外力加速度的情况下，能准确输出ROLL/PITCH两轴姿态角度 并且此角度不会有累积误差，在更长的时间尺度内都是准确的。但是加速度传感器测角度的缺点是加速度传感器实际上是用MEMS技术检测惯性力造成的微小形变，而惯性力与重力本质是一样的,所以加速度计就不会区分重力加速度与外力加速度，当系统在三维空间做变速运动时，它的输出就不正确了。
陀螺仪输出角速度是瞬时量，角速度在姿态平衡上不能直接使用， 需要角速度与时间积分计算角度，得到的角度变化量与初始角度相加，就得到目标角度，其中积分时间Dt越小输出的角度越精确。但陀螺仪的原理决定了它的测量基准是自身，并没有系统外的绝对参照物，加上Dt是不可能无限小的，所以积分的累积误差会随着时间的流逝迅速增加，最终导致输出角度与实际不符，所以陀螺仪只能工作在相对较短的时间尺度内[1]  。
所以在没有其它参照物的基础上，要得到较为真实的姿态角，就要利用加权算法扬长避短，结合两者的优点，摈弃其各自缺点,设计算法在短时间尺度内增加陀螺仪的权值，在更长时间尺度内增加加速度权值，这样系统输出角度就接近真实值了。


roll/pitch does not drift,though yaw does：
如果机器人移动缓慢，那么加速度测量影响因素直接来源于G/g：
f=ma，a=f/m，m=G（重力）/g 。（9.8N/kg)



*/