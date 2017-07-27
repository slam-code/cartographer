
#include "cartographer/mapping/sparse_pose_graph/constraint_builder.h"

#include "cartographer/mapping_2d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_3d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/sensor/voxel_filter.h"

namespace cartographer {
  namespace mapping {
    namespace sparse_pose_graph {
//参数值可以在sparse_pose_graph.lua查看。
      proto::ConstraintBuilderOptions CreateConstraintBuilderOptions(
        common::LuaParameterDictionary* const parameter_dictionary) {
        proto::ConstraintBuilderOptions options;
        options.set_sampling_ratio(parameter_dictionary->GetDouble("sampling_ratio"));
        options.set_max_constraint_distance(
          parameter_dictionary->GetDouble("max_constraint_distance"));
        *options.mutable_adaptive_voxel_filter_options() =
        sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary->GetDictionary("adaptive_voxel_filter").get());
        options.set_min_score(parameter_dictionary->GetDouble("min_score"));
        options.set_global_localization_min_score(
          parameter_dictionary->GetDouble("global_localization_min_score"));
        options.set_loop_closure_translation_weight(
          parameter_dictionary->GetDouble("loop_closure_translation_weight"));
        options.set_loop_closure_rotation_weight(
          parameter_dictionary->GetDouble("loop_closure_rotation_weight"));
        options.set_log_matches(parameter_dictionary->GetBool("log_matches"));
        *options.mutable_fast_correlative_scan_matcher_options() =
        mapping_2d::scan_matching::CreateFastCorrelativeScanMatcherOptions(
          parameter_dictionary->GetDictionary("fast_correlative_scan_matcher")
          .get());
        *options.mutable_ceres_scan_matcher_options() =
        mapping_2d::scan_matching::CreateCeresScanMatcherOptions(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
        *options.mutable_fast_correlative_scan_matcher_options_3d() =
        mapping_3d::scan_matching::CreateFastCorrelativeScanMatcherOptions(
          parameter_dictionary
          ->GetDictionary("fast_correlative_scan_matcher_3d")
          .get());
        *options.mutable_ceres_scan_matcher_options_3d() =
        mapping_3d::scan_matching::CreateCeresScanMatcherOptions(
          parameter_dictionary->GetDictionary("ceres_scan_matcher_3d").get());
        return options;
      }

}  // namespace sparse_pose_graph
}  // namespace mapping
}  // namespace cartographer

/*
sparse_pose_graph.lua参数：
 constraint_builder = {
    sampling_ratio = 0.3,
    max_constraint_distance = 15.,
    adaptive_voxel_filter = {
      max_length = 0.9,
      min_num_points = 100,
      max_range = 50.,
    },
    min_score = 0.55,
    global_localization_min_score = 0.6,
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
    log_matches = true,
    fast_correlative_scan_matcher = {
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,
    },
    ceres_scan_matcher = {
      occupied_space_weight = 20.,
      translation_weight = 10.,
      rotation_weight = 1.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      rotational_histogram_size = 120,
      min_rotational_score = 0.77,
      linear_xy_search_window = 4.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 20.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },

*/