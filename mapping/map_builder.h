
#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer/mapping_2d/sparse_pose_graph.h"
#include "cartographer/mapping_3d/sparse_pose_graph.h"
#include "cartographer/sensor/collator.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

/*
MapBuilder类,建图,不可拷贝/赋值


MapBuilder类和TrajectoryBuilder类即真正的开始重建局部子图submaps,
并且采集稀疏位姿图用于闭环检测。

1)构造函数根据options初始化数据成员，包括线程数量和sparse_pose_graph_options。


Wires up（为…） 接通电源;

Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
and a SparsePoseGraph for loop closure.

*/
class MapBuilder {
 public:
  MapBuilder(const proto::MapBuilderOptions& options);
  ~MapBuilder();

  MapBuilder(const MapBuilder&) = delete;
  MapBuilder& operator=(const MapBuilder&) = delete;

  // Create a new trajectory and return its index.
  //根据传感器id和options新建一个轨迹线，返回轨迹线的索引
  int AddTrajectoryBuilder(
      const std::unordered_set<string>& expected_sensor_ids,
      const proto::TrajectoryBuilderOptions& trajectory_options);

  // Returns the TrajectoryBuilder corresponding to the specified
  // 'trajectory_id'.
  //根据轨迹id返回指向该轨迹的TrajectoryBuilder对象指针。
  mapping::TrajectoryBuilder* GetTrajectoryBuilder(int trajectory_id) const;

  // Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
  // i.e. no further sensor data is expected.
  //标记该轨迹已完成data采集，后续不再接收data
  void FinishTrajectory(int trajectory_id);

  // Must only be called if at least one unfinished trajectory exists. Returns
  // the ID of the trajectory that needs more data before the MapBuilder is
  // unblocked.
  //阻塞的轨迹，常见于该条轨迹上的传感器迟迟不提交data。
  int GetBlockingTrajectoryId() const;

  // Returns the trajectory connectivity.获得一系列轨迹的连通域
  proto::TrajectoryConnectivity GetTrajectoryConnectivity();

  // Fills the SubmapQuery::Response corresponding to 'submap_index' from
  // 'trajectory_id'. Returns an error string on failure, or an empty string on
  // success.
  //把轨迹id和子图索引对应的submap，序列化到文件
  string SubmapToProto(int trajectory_id, int submap_index,
                       proto::SubmapQuery::Response* response);

  //在建图的轨迹数量
  int num_trajectory_builders() const;

  mapping::SparsePoseGraph* sparse_pose_graph();

 private:
  const proto::MapBuilderOptions options_; // 建图选项
  common::ThreadPool thread_pool_;         //线程数量，不可变。

  std::unique_ptr<mapping_2d::SparsePoseGraph> sparse_pose_graph_2d_; //稀疏2D图
  std::unique_ptr<mapping_3d::SparsePoseGraph> sparse_pose_graph_3d_; //稀疏3D图
  mapping::SparsePoseGraph* sparse_pose_graph_;  //稀疏位姿

  sensor::Collator sensor_collator_;  //收集传感器采集的数据
  std::vector<std::unique_ptr<mapping::TrajectoryBuilder>> trajectory_builders_;//轨迹线集合
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
