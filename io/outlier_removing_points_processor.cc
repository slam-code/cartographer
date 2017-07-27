

#include "cartographer/io/outlier_removing_points_processor.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
/*
VOXEL_SIZE = 5e-2
*/
std::unique_ptr<OutlierRemovingPointsProcessor>
OutlierRemovingPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<OutlierRemovingPointsProcessor>(
      dictionary->GetDouble("voxel_size"), next); //构造一个对象,返回一个智能指针
}

/*
构造函数,传递一个 voxel_size 和  PointsProcessor* next
*/
OutlierRemovingPointsProcessor::OutlierRemovingPointsProcessor(
    const double voxel_size, PointsProcessor* next)
    : voxel_size_(voxel_size),
      next_(next),
      state_(State::kPhase1),
      voxels_(voxel_size_) {
  LOG(INFO) << "Marking hits...";
}

/*
根据3个不同的state分别处理 points
*/
void OutlierRemovingPointsProcessor::Process(
    std::unique_ptr<PointsBatch> points) {
  switch (state_) {
    case State::kPhase1:
      ProcessInPhaseOne(*points);
      break;

    case State::kPhase2:
      ProcessInPhaseTwo(*points);
      break;

    case State::kPhase3:
      ProcessInPhaseThree(std::move(points));
      break;
  }
}

/*
更新state,并返回 FlushResult结果
*/
PointsProcessor::FlushResult OutlierRemovingPointsProcessor::Flush() {
  switch (state_) {
    case State::kPhase1:
      LOG(INFO) << "Counting rays...";
      state_ = State::kPhase2;
      return FlushResult::kRestartStream;

    case State::kPhase2:
      LOG(INFO) << "Filtering outliers...";
      state_ = State::kPhase3;
      return FlushResult::kRestartStream;

    case State::kPhase3:
      CHECK(next_->Flush() == FlushResult::kFinished)
          << "Voxel filtering and outlier removal must be configured to occur "
             "after any stages that require multiple passes.";
             // multiple passes：多次传输。
      return FlushResult::kFinished;
  }
  LOG(FATAL);
}


/*
state 1，统计光线的数量。
*/
void OutlierRemovingPointsProcessor::ProcessInPhaseOne(
    const PointsBatch& batch) {
  for (size_t i = 0; i < batch.points.size(); ++i) {
    ++voxels_.mutable_value(voxels_.GetCellIndex(batch.points[i]))->hits;
  }
}

/*


*/
void OutlierRemovingPointsProcessor::ProcessInPhaseTwo(
    const PointsBatch& batch) {
  // TODO(whess): This samples every 'voxel_size' distance and could be improved
  // by better ray casting, and also by marking the hits of the current range
  // data to be excluded.
  for (size_t i = 0; i < batch.points.size(); ++i) {
    const Eigen::Vector3f delta = batch.points[i] - batch.origin;
    const float length = delta.norm();
    for (float x = 0; x < length; x += voxel_size_) {
      const auto index =
          voxels_.GetCellIndex(batch.origin + (x / length) * delta);
      if (voxels_.value(index).hits > 0) {
        ++voxels_.mutable_value(index)->rays;
      }
    }
  }
}

void OutlierRemovingPointsProcessor::ProcessInPhaseThree(
    std::unique_ptr<PointsBatch> batch) {
  constexpr double kMissPerHitLimit = 3;
  std::vector<int> to_remove;
  for (size_t i = 0; i < batch->points.size(); ++i) {
    const auto voxel = voxels_.value(voxels_.GetCellIndex(batch->points[i]));
    if (!(voxel.rays < kMissPerHitLimit * voxel.hits)) {
      to_remove.push_back(i);
    }
  }
  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));
}

}  // namespace io
}  // namespace cartographer
