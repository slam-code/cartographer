
#ifndef CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
#define CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.pb.h"

namespace cartographer {
namespace mapping {
namespace sparse_pose_graph {

//获取sparse_pose_graph.lua配置参数
proto::ConstraintBuilderOptions CreateConstraintBuilderOptions(
    common::LuaParameterDictionary* parameter_dictionary);

}  // namespace sparse_pose_graph
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
