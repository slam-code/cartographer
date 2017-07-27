
#include "cartographer/mapping/odometry_state_tracker.h"

#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

OdometryState::OdometryState(const common::Time time,
                             const transform::Rigid3d& odometer_pose,
                             const transform::Rigid3d& state_pose)
    : time(time), odometer_pose(odometer_pose), state_pose(state_pose) {}

OdometryStateTracker::OdometryStateTracker(const int window_size)
    : window_size_(window_size) {
  CHECK_GT(window_size, 0);
}

const OdometryStateTracker::OdometryStates&
OdometryStateTracker::odometry_states() const {
  return odometry_states_;
}

//添新删旧
void OdometryStateTracker::AddOdometryState(
    const OdometryState& odometry_state) {
  odometry_states_.push_back(odometry_state);
  while (odometry_states_.size() > window_size_) {
    odometry_states_.pop_front();
  }
}

bool OdometryStateTracker::empty() const { return odometry_states_.empty(); }

//返回最后一个
const OdometryState& OdometryStateTracker::newest() const {
  return odometry_states_.back();
}

}  // namespace mapping
}  // namespace cartographer
