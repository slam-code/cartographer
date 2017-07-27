
#include "cartographer/io/counting_points_processor.h"

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

//初始化是num是0
CountingPointsProcessor::CountingPointsProcessor(PointsProcessor* next)
    : num_points_(0), next_(next) {}

//与其他FromDictionary()函数统一写法
std::unique_ptr<CountingPointsProcessor>
CountingPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<CountingPointsProcessor>(next);
}

//不处理points,而是将num_points_加上batch.size(),，即统计点云数据。然后直接流水线到下一PointsProcessor
void CountingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  num_points_ += batch->points.size();//相加
  next_->Process(std::move(batch));
}

//依据下一阶段的状态重置本阶段的状态。
PointsProcessor::FlushResult CountingPointsProcessor::Flush() {
  switch (next_->Flush()) {
    case FlushResult::kFinished:
      LOG(INFO) << "Processed " << num_points_ << " and finishing.";
      return FlushResult::kFinished;

    case FlushResult::kRestartStream:
      LOG(INFO) << "Processed " << num_points_ << " and restarting stream.";
      num_points_ = 0; //重启,则置为0
      return FlushResult::kRestartStream;
  }
  LOG(FATAL);
}

}  // namespace io
}  // namespace cartographer
