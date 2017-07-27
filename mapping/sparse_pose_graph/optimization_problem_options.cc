
#include "cartographer/mapping/sparse_pose_graph/optimization_problem_options.h"

#include "cartographer/common/ceres_solver_options.h"

namespace cartographer {
namespace mapping {
namespace sparse_pose_graph {
/*
sparse_pose_graph.lua:

optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1e3,
    rotation_weight = 3e5,
    consecutive_scan_translation_penalty_factor = 1e5,
    consecutive_scan_rotation_penalty_factor = 1e5,
    log_solver_summary = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },

参数配置如上。
*/
proto::OptimizationProblemOptions CreateOptimizationProblemOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::OptimizationProblemOptions options;
  options.set_huber_scale(parameter_dictionary->GetDouble("huber_scale"));
  options.set_acceleration_weight(
      parameter_dictionary->GetDouble("acceleration_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_consecutive_scan_translation_penalty_factor(
      parameter_dictionary->GetDouble(
          "consecutive_scan_translation_penalty_factor"));
  options.set_consecutive_scan_rotation_penalty_factor(
      parameter_dictionary->GetDouble(
          "consecutive_scan_rotation_penalty_factor"));
  options.set_log_solver_summary(
      parameter_dictionary->GetBool("log_solver_summary"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

}  // namespace sparse_pose_graph
}  // namespace mapping
}  // namespace cartographer
