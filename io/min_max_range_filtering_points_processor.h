
#ifndef CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {
/*

PointsProcessor的第一个子类(1).
功能:继承自PointsProcessor点云虚基类.距离过滤器,L2 距离不在Min-Max范围内的都过滤掉.
数据成员:
1),min_range_ 最小范围
2),max_range_ 最大范围
3),PointsProcessor* const next_:完成本轮Processor,接下来进行下一次Processor.
*/

// Filters all points that are farther away from their 'origin' as 'max_range'
// or closer than 'min_range'.
class MinMaxRangeFiteringPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "min_max_range_filter";

   
  MinMaxRangeFiteringPointsProcessor(double min_range, double max_range,
                                     PointsProcessor* next);

  //从.lua文件加载min_range和max_range,如:0-30 
  static std::unique_ptr<MinMaxRangeFiteringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~MinMaxRangeFiteringPointsProcessor() override {}//覆盖

  MinMaxRangeFiteringPointsProcessor(
      const MinMaxRangeFiteringPointsProcessor&) = delete;
  MinMaxRangeFiteringPointsProcessor& operator=(
      const MinMaxRangeFiteringPointsProcessor&) = delete;

//点云处理,删除[min,max]以外的point，并把数据传递到下一轮Processor处理。
  void Process(std::unique_ptr<PointsBatch> batch) override;

  FlushResult Flush() override;
  //调用next_->Flush();父类调用Flush(),但在运行时才会决定是否调用子类的Flush()

 private:
  const double min_range_;
  const double max_range_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_
