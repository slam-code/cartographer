
#ifndef CARTOGRAPHER_IO_FIXED_RATIO_SAMPLING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_FIXED_RATIO_SAMPLING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

/*
FixedRatioSamplingPointsProcessor是PointsProcessor的第七个子类(7).
FixedRatioSamplingPointsProcessor是采样过滤器,
该class只让具有固定采样频率的点通过,其余全部 remove.

sampling_ratio=1,即无任何操作,全通滤波.

sparse_pose_graph.lua:
 sampling_ratio = 0.3,

*/
// Only let a fixed 'sampling_ratio' of points through. A 'sampling_ratio' of 1.
// makes this filter a no-op.
class FixedRatioSamplingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "fixed_ratio_sampler";

  FixedRatioSamplingPointsProcessor(double sampling_ratio,
                                    PointsProcessor* next);

  static std::unique_ptr<FixedRatioSamplingPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~FixedRatioSamplingPointsProcessor() override{};

  FixedRatioSamplingPointsProcessor(const FixedRatioSamplingPointsProcessor&) =
      delete;
  FixedRatioSamplingPointsProcessor& operator=(
      const FixedRatioSamplingPointsProcessor&) = delete;

//根据采样率采集点云，不在采样点上的全部删除。
  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const double sampling_ratio_;//采样率
  PointsProcessor* const next_;
  std::unique_ptr<common::FixedRatioSampler> sampler_;//具有固定采样率的采样函数
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_FIXED_RATIO_SAMPLING_POINTS_PROCESSOR_H_
