
#ifndef CARTOGRAPHER_IO_COLORING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_COLORING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {
/*
  {
      action = "color_points",
      frame_id = "horizontal_laser_link",
      color = { 255., 0., 0. },
    },
    {
      action = "color_points",
      frame_id = "vertical_laser_link",
      color = { 0., 255., 0. },
    },

ColoringPointsProcessor是PointsProcessor的第六子类(6).
功能: 用固定的Color填充PointsBatch的Color分量。

数据成员:
1),color_：rgb值
2),frame_id_:只有相同的id才填充Color，color一般为[255,0,0],[0,255,0]
2),next_下一阶段的PointsProcessor.

*/

// Colors points with a fixed color by frame_id.
class ColoringPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "color_points";

  ColoringPointsProcessor(const Color& color, const string& frame_id,
                          PointsProcessor* next);

  static std::unique_ptr<ColoringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~ColoringPointsProcessor() override{};

  ColoringPointsProcessor(const ColoringPointsProcessor&) = delete;
  ColoringPointsProcessor& operator=(const ColoringPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const Color color_;
  const string frame_id_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_COLORING_POINTS_PROCESSOR_H_
