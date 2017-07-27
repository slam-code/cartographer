
#ifndef CARTOGRAPHER_IO_COUNTING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_COUNTING_POINTS_PROCESSOR_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

/*

PointsProcessor的第二个子类(2).
功能:记录有多少 point被输出处理

数据成员:
1),num_points_记录points数量
2),next_下一阶段的PointsProcessor.

*/
// Passes through points, but keeps track of how many points it saw and output
// that on Flush.
class CountingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "dump_num_points";
  explicit CountingPointsProcessor(PointsProcessor* next);

  static std::unique_ptr<CountingPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~CountingPointsProcessor() override {}

  CountingPointsProcessor(const CountingPointsProcessor&) = delete;
  CountingPointsProcessor& operator=(const CountingPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> points) override;
  FlushResult Flush() override;

 private:
  int64 num_points_;
  PointsProcessor* next_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_COUNTING_POINTS_PROCESSOR_H_
