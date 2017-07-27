#ifndef CARTOGRAPHER_IO_XRAY_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_XRAY_POINTS_PROCESSOR_H_

#include <map>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/detect_floors.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/transform/rigid_transform.h"

/*配置文件概览：
VOXEL_SIZE = 5e-2

YZ_TRANSFORM =  {
  translation = { 0., 0., 0. },
  rotation = { 0. , 0., math.pi, },
}

{
    action = "write_xray_image",
    voxel_size = VOXEL_SIZE,
    filename = "xray_yz_all_intensity",
    transform = YZ_TRANSFORM,
},

*/

namespace cartographer {
namespace io {
/*
xray_points_processor.h是PointsProcessor的第三个子类(3).
功能:将x射线以体素为voxel_size写入image。

数据成员:
1), 
2), 

*/
// Creates X-ray cuts through the points with pixels being 'voxel_size' big.
class XRayPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "write_xray_image";
  XRayPointsProcessor(double voxel_size, const transform::Rigid3f& transform,
                      const std::vector<mapping::Floor>& floors,
                      const string& output_filename,
                      FileWriterFactory file_writer_factory,
                      PointsProcessor* next);

  static std::unique_ptr<XRayPointsProcessor> FromDictionary(
      const mapping::proto::Trajectory& trajectory,
      FileWriterFactory file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~XRayPointsProcessor() override {}

//将点云写入.png文件，然后流水到下一处理器
  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

  Eigen::AlignedBox3i bounding_box() const { return bounding_box_; }

 private:
  struct ColumnData {
    double sum_r = 0.;
    double sum_g = 0.;
    double sum_b = 0.;
    uint32_t count = 0;
  };

  struct Aggregation {
    mapping_3d::HybridGridBase<bool> voxels;
    std::map<std::pair<int, int>, ColumnData> column_data;
  };

  void WriteVoxels(const Aggregation& aggregation,
                   FileWriter* const file_writer);
  void Insert(const PointsBatch& batch, const transform::Rigid3f& transform,
              Aggregation* aggregation);

  PointsProcessor* const next_;
  FileWriterFactory file_writer_factory_; //负责文件写入的工厂

  // If empty, we do not separate into floors.
  std::vector<mapping::Floor> floors_; //楼层

  const string output_filename_;       //文件名称
  const transform::Rigid3f transform_; //点云涉及的变换

  // Only has one entry if we do not separate into floors.
  std::vector<Aggregation> aggregations_; //聚集数据

  // Bounding box containing all cells with data in all 'aggregations_'.
  Eigen::AlignedBox3i bounding_box_; //边界框
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_XRAY_POINTS_PROCESSOR_H_
