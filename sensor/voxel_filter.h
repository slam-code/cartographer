
#ifndef CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"

/*

message AdaptiveVoxelFilterOptions { 
  optional float max_length = 1; 
  optional float min_num_points = 2;
  optional float max_range = 3;
}

2d:
adaptive_voxel_filter = {
      max_length = 0.9,     //voxel_的大小edge的最大值
      min_num_points = 100, //voxel_最多“占据”的points数量
      max_range = 50.,
    },


Voxel, 三维像素,体素
3D模型体素化：http://blog.csdn.net/bugrunner/article/details/5527332

 */
namespace cartographer {
namespace sensor {


//返回过滤后的点云,size是体素的长度。
// Returns a voxel filtered copy of 'point_cloud' where 'size' is the length
// a voxel edge.
PointCloud VoxelFiltered(const PointCloud& point_cloud, float size);


/*

体素滤波器,对每一个体素voxel,采用第一个point代替所有的points

VoxelFilter:不可拷贝/不可赋值
默认构造函数指定体素边界大小



*/
// Voxel filter for point clouds. For each voxel, the assembled point cloud
// contains the first point that fell into it from any of the inserted point
// clouds.
class VoxelFilter {
 public:
  // 'size' is the length of a voxel edge.
  explicit VoxelFilter(float size);

  VoxelFilter(const VoxelFilter&) = delete;
  VoxelFilter& operator=(const VoxelFilter&) = delete;

  //将点云插入体素网格中
  // Inserts a point cloud into the voxel filter.
  void InsertPointCloud(const PointCloud& point_cloud);

  //返回表达occupied 体素的点云
  // Returns the filtered point cloud representing the occupied voxels.
  const PointCloud& point_cloud() const;

 private:
  mapping_3d::HybridGridBase<uint8> voxels_;//以体素表示的网格，Grid/Pixel
  PointCloud point_cloud_;  //网格内的点云
};

/*
自适应体素滤波,不可拷贝/赋值

*/
proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class AdaptiveVoxelFilter {
 public:
  //根据配置文件设置自适应体素滤波的options
  explicit AdaptiveVoxelFilter(
      const proto::AdaptiveVoxelFilterOptions& options);

  AdaptiveVoxelFilter(const AdaptiveVoxelFilter&) = delete;
  AdaptiveVoxelFilter& operator=(const AdaptiveVoxelFilter&) = delete;

  //对点云进行体素滤波,返回过滤后的点云
  PointCloud Filter(const PointCloud& point_cloud) const;

 private:
  const proto::AdaptiveVoxelFilterOptions options_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_
