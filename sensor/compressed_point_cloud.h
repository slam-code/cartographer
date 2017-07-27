
#ifndef CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_

#include <iterator>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace sensor {

/*
CompressedPointCloud是点云压缩类,
目的：压缩ponits以减少存储空间，压缩后有精度损失。
方法：按照block分组。

只有一个私有的
*/
// A compressed representation of a point cloud consisting of a collection of
// points (Vector3f).
// Internally, points are grouped by blocks. Each block encodes a bit of meta
// data (number of points in block, coordinates of the block) and encodes each
// point with a fixed bit rate in relation to the block.
class CompressedPointCloud {
 public:
  class ConstIterator; //前置声明

  CompressedPointCloud() : num_points_(0) {}
  explicit CompressedPointCloud(const PointCloud& point_cloud);

  // Returns decompressed point cloud.
  PointCloud Decompress() const;

  bool empty() const;                   // num_points_==0
  size_t size() const;                  // num_points_
  ConstIterator begin() const;
  ConstIterator end() const;

  proto::CompressedPointCloud ToProto() const;

 private:
  CompressedPointCloud(const std::vector<int32>& point_data, size_t num_points);

  std::vector<int32> point_data_;
  size_t num_points_;
};

/*前行迭代器*/
// Forward iterator for compressed point clouds.
class CompressedPointCloud::ConstIterator
    : public std::iterator<std::forward_iterator_tag, Eigen::Vector3f> {
 public:
  // Creates begin iterator.
  explicit ConstIterator(const CompressedPointCloud* compressed_point_cloud);

  // Creates end iterator.
  static ConstIterator EndIterator(
      const CompressedPointCloud* compressed_point_cloud);

  Eigen::Vector3f operator*() const;
  ConstIterator& operator++();
  bool operator!=(const ConstIterator& it) const;

 private:
  // Reads next point from buffer. Also handles reading the meta data of the
  // next block, if the current block is depleted.
  void ReadNextPoint();

  const CompressedPointCloud* compressed_point_cloud_;
  size_t remaining_points_;
  int32 remaining_points_in_current_block_;
  Eigen::Vector3f current_point_;
  Eigen::Vector3i current_block_coordinates_;
  std::vector<int32>::const_iterator input_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_
