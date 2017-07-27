
#include "cartographer/mapping/map_builder.h"

#include <cmath>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/collated_trajectory_builder.h"
#include "cartographer/mapping_2d/global_trajectory_builder.h"
#include "cartographer/mapping_3d/global_trajectory_builder.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

//backpack_3d.lua/revo_lds.lua/taurob_tracker.lua
/*

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = false,
  use_laser_scan = false,
  use_multi_echo_laser_scan = false,
  num_point_clouds = 2,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
}

TRAJECTORY_BUILDER_3D.scans_per_accumulation = 160

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
MAP_BUILDER.sparse_pose_graph.optimization_problem.huber_scale = 5e2
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 320
MAP_BUILDER.sparse_pose_graph.constraint_builder.sampling_ratio = 0.03
MAP_BUILDER.sparse_pose_graph.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- Reuse the coarser 3D voxel filter to speed up the computation of loop closure
-- constraints.
MAP_BUILDER.sparse_pose_graph.constraint_builder.adaptive_voxel_filter = TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter
MAP_BUILDER.sparse_pose_graph.constraint_builder.min_score = 0.62

return options

*/
proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::MapBuilderOptions options;
  options.set_use_trajectory_builder_2d(
      parameter_dictionary->GetBool("use_trajectory_builder_2d"));//false
  options.set_use_trajectory_builder_3d(
      parameter_dictionary->GetBool("use_trajectory_builder_3d"));//true
  options.set_num_background_threads(
      parameter_dictionary->GetNonNegativeInt("num_background_threads"));//7

  //sparse_pose_graph.lua
  *options.mutable_sparse_pose_graph_options() = CreateSparsePoseGraphOptions(
      parameter_dictionary->GetDictionary("sparse_pose_graph").get());
  CHECK_NE(options.use_trajectory_builder_2d(),
           options.use_trajectory_builder_3d());
  return options;
}

MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
  if (options.use_trajectory_builder_2d()) {
    sparse_pose_graph_2d_ = common::make_unique<mapping_2d::SparsePoseGraph>(
        options_.sparse_pose_graph_options(), &thread_pool_);
    sparse_pose_graph_ = sparse_pose_graph_2d_.get();
  }
  if (options.use_trajectory_builder_3d()) {
    sparse_pose_graph_3d_ = common::make_unique<mapping_3d::SparsePoseGraph>(
        options_.sparse_pose_graph_options(), &thread_pool_);
    sparse_pose_graph_ = sparse_pose_graph_3d_.get();
  }
}

MapBuilder::~MapBuilder() {}

int MapBuilder::AddTrajectoryBuilder(
    const std::unordered_set<string>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options) {
  const int trajectory_id = trajectory_builders_.size();
  if (options_.use_trajectory_builder_3d()) {
    CHECK(trajectory_options.has_trajectory_builder_3d_options());
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            &sensor_collator_, trajectory_id, expected_sensor_ids,
            common::make_unique<mapping_3d::GlobalTrajectoryBuilder>(
                trajectory_options.trajectory_builder_3d_options(),
                trajectory_id, sparse_pose_graph_3d_.get())));
  } else {
    CHECK(trajectory_options.has_trajectory_builder_2d_options());
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            &sensor_collator_, trajectory_id, expected_sensor_ids,
            common::make_unique<mapping_2d::GlobalTrajectoryBuilder>(
                trajectory_options.trajectory_builder_2d_options(),
                trajectory_id, sparse_pose_graph_2d_.get())));
  }
  return trajectory_id;
}

TrajectoryBuilder* MapBuilder::GetTrajectoryBuilder(
    const int trajectory_id) const {
  return trajectory_builders_.at(trajectory_id).get();
}

void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_.FinishTrajectory(trajectory_id);
}

int MapBuilder::GetBlockingTrajectoryId() const {
  return sensor_collator_.GetBlockingTrajectoryId();
}

proto::TrajectoryConnectivity MapBuilder::GetTrajectoryConnectivity() {
  return ToProto(sparse_pose_graph_->GetConnectedTrajectories());
}

string MapBuilder::SubmapToProto(const int trajectory_id,
                                 const int submap_index,
                                 proto::SubmapQuery::Response* const response) {
  if (trajectory_id < 0 || trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " + std::to_string(trajectory_id) +
           " but there are only " + std::to_string(num_trajectory_builders()) +
           " trajectories.";
  }

  const std::vector<transform::Rigid3d> submap_transforms =
      sparse_pose_graph_->GetSubmapTransforms(trajectory_id);
  if (submap_index < 0 ||
      static_cast<size_t>(submap_index) >= submap_transforms.size()) {
    return "Requested submap " + std::to_string(submap_index) +
           " from trajectory " + std::to_string(trajectory_id) +
           " but there are only " + std::to_string(submap_transforms.size()) +
           " submaps in this trajectory.";
  }

  const Submaps* const submaps =
      trajectory_builders_.at(trajectory_id)->submaps();
  response->set_submap_version(submaps->Get(submap_index)->num_range_data);
  submaps->SubmapToProto(submap_index, submap_transforms[submap_index],
                         response);
  return "";
}

int MapBuilder::num_trajectory_builders() const {
  return trajectory_builders_.size();
}

SparsePoseGraph* MapBuilder::sparse_pose_graph() { return sparse_pose_graph_; }

}  // namespace mapping
}  // namespace cartographer
