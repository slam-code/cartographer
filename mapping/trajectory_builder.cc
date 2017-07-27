
#include "cartographer/mapping/trajectory_builder.h"

#include "cartographer/mapping_2d/local_trajectory_builder_options.h"
#include "cartographer/mapping_3d/local_trajectory_builder_options.h"

namespace cartographer {
namespace mapping {

//获取trajectory_builder.lua的内容
proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::TrajectoryBuilderOptions options;
  *options.mutable_trajectory_builder_2d_options() =
      mapping_2d::CreateLocalTrajectoryBuilderOptions(
          parameter_dictionary->GetDictionary("trajectory_builder_2d").get());
  *options.mutable_trajectory_builder_3d_options() =
      mapping_3d::CreateLocalTrajectoryBuilderOptions(
          parameter_dictionary->GetDictionary("trajectory_builder_3d").get());
  return options;
}

}  // namespace mapping
}  // namespace cartographer
