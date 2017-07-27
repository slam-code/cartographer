
#include "cartographer/io/coloring_points_processor.h"

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
/*
根据assets_writer_backpack_2d.lua配置:.lua文件概览:
-- Now we recolor our points by frame and write another batch of X-Rays. It
-- is visible in them what was seen by the horizontal and the vertical
-- laser.
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
FromDictionary()根据.lua配置文件获取frame_id下color的{r,g,b}
*/


std::unique_ptr<ColoringPointsProcessor>
ColoringPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
/*
frame_id:horizontal_laser_link或者vertical_laser_link
color:{ 255., 0., 0. },或者{ 0., 255., 0. },
*/
  const string frame_id = dictionary->GetString("frame_id");
  const std::vector<double> color_values =
      dictionary->GetDictionary("color")->GetArrayValuesAsDoubles();
  const Color color = {{static_cast<uint8_t>(color_values[0]),
                        static_cast<uint8_t>(color_values[1]),
                        static_cast<uint8_t>(color_values[2])}};
  return common::make_unique<ColoringPointsProcessor>(color, frame_id, next);
}

ColoringPointsProcessor::ColoringPointsProcessor(const Color& color,
                                                 const string& frame_id,
                                                 PointsProcessor* const next)
    : color_(color), frame_id_(frame_id), next_(next) {}

/*
只对相同的frame_id_处理：着色
*/
void ColoringPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (batch->frame_id == frame_id_) {
    batch->colors.clear();
    for (size_t i = 0; i < batch->points.size(); ++i) {
      batch->colors.push_back(color_);//用color 填充batch的colors
    }
  }
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult ColoringPointsProcessor::Flush() {
  return next_->Flush();
}

}  // namespace io
}  // namespace cartographer
