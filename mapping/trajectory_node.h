
#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_

#include <deque>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

class Submaps;
/*
轨迹节点TrajectoryNode类，含有一个内部类ConstantData。
TrajectoryNode作用：在连续的轨迹上采样一些离散的点用于key frame，标识pose frame。

数据成员：
1), ConstantData* 
2) 位姿，pose


ConstantData类：

数据成员：
1，time ：时间
2，range_data_2d和range_data_3d：测量数据
3,trajectory_id:本节点所属的轨迹
4，Rigid3d ：tracking frame 到 pose frame的矩阵变换。
*/
struct TrajectoryNode {

  struct ConstantData {
    common::Time time;

    // Range data in 'pose' frame. Only used in the 2D case.
    sensor::RangeData range_data_2d;//测量得到的2D range数据

    // Range data in 'pose' frame. Only used in the 3D case.
    sensor::CompressedRangeData range_data_3d;//测量得到的3D range数据

    // Trajectory this node belongs to.
    int trajectory_id; //节点所属id

    // Transform from the 3D 'tracking' frame to the 'pose' frame of the range
    // data, which contains roll, pitch and height for 2D. In 3D this is always
    // identity.
    transform::Rigid3d tracking_to_pose; //涉及的3D变换
  };

  common::Time time() const { return constant_data->time; }

  const ConstantData* constant_data;
  //常指针.指向某块内存,该内存块的数据不变，指针本身可以变。

  transform::Rigid3d pose;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
