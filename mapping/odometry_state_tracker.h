
#ifndef CARTOGRAPHER_MAPPING_ODOMETRY_STATE_TRACKER_H_
#define CARTOGRAPHER_MAPPING_ODOMETRY_STATE_TRACKER_H_

#include <deque>

#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {
/*
.lua设置：
imu_gravity_time_constant = 10.,
num_odometry_states = 1000,

OdometryState类含3个数据成员
1,time，时间
2,Rigid3d，里程计的位置
3,Rigid3d，状态位置

*/
struct OdometryState {
  OdometryState(common::Time time, const transform::Rigid3d& odometer_pose,
                const transform::Rigid3d& state_pose);
  OdometryState() {}

  common::Time time = common::Time::min();
  transform::Rigid3d odometer_pose = transform::Rigid3d::Identity();
  transform::Rigid3d state_pose = transform::Rigid3d::Identity();
};

/*
OdometryStateTracker:记录跟踪里程计的多个状态类,maxsize :window_size
含义2个数据成员
1,OdometryStates，记录多个里程计状态，是一个双端队列deque
2,window_size_,滑动窗大小,即队列大小
成员函数：
1，构造函数初始化滑动窗大小.
2，AddOdometryState()添加一个新的里程计状态，若deque已满则删除旧有的States


*/
// Keeps track of the odometry states by keeping sliding window over some
// number of them.
class OdometryStateTracker {
 public:
  using OdometryStates = std::deque<OdometryState>;

  explicit OdometryStateTracker(int window_size);

  const OdometryStates& odometry_states() const;

/*添加新的里程计状态,超出滑动窗大小时,旧的删除*/
  // Adds a new 'odometry_state' and makes sure the maximum number of previous
  // odometry states is not exceeded.
  void AddOdometryState(const OdometryState& odometry_state);

  // Returns true if no elements are present in the odometry queue.
  bool empty() const;

  // Retrieves the most recent OdometryState. Must not be called when empty.
  const OdometryState& newest() const;

 private:
  OdometryStates odometry_states_;
  size_t window_size_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_ODOMETRY_STATE_TRACKER_H_
