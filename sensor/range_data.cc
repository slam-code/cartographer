
#include "cartographer/sensor/range_data.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {
/*

message PointCloud {     
  repeated float x = 3 [packed = true];
  repeated float y = 4 [packed = true];
  repeated float z = 5 [packed = true];
}

message CompressedPointCloud { 
  optional int32 num_points = 1;
  repeated int32 point_data = 3 [packed = true];
}

message RangeData {             
  optional transform.proto.Vector3f origin = 1;
  optional PointCloud point_cloud = 2;
  optional PointCloud missing_echo_point_cloud = 3;
}

PointCloud;//vector，元素是3*1f

*/
proto::RangeData ToProto(const RangeData& range_data) {
  proto::RangeData proto;
  *proto.mutable_origin() = transform::ToProto(range_data.origin);
  *proto.mutable_point_cloud() = ToProto(range_data.returns);
  *proto.mutable_missing_echo_point_cloud() = ToProto(range_data.misses);
  return proto;
}

RangeData FromProto(const proto::RangeData& proto) {
  auto range_data = RangeData{
      transform::ToEigen(proto.origin()), ToPointCloud(proto.point_cloud()),
      ToPointCloud(proto.missing_echo_point_cloud()),
  };
  return range_data;
}

/*

将sensor坐标变换为机器坐标。
*/
RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform) {
  return RangeData{
      transform * range_data.origin,
      TransformPointCloud(range_data.returns, transform),
      TransformPointCloud(range_data.misses, transform),
  };
}

/*
不在给定的z轴范围内的点云删除。
*/
RangeData CropRangeData(const RangeData& range_data, const float min_z,
                        const float max_z) {
  return RangeData{range_data.origin, Crop(range_data.returns, min_z, max_z),
                   Crop(range_data.misses, min_z, max_z)};
}

/*
压缩，有精度丢失。
*/
CompressedRangeData Compress(const RangeData& range_data) {
  return CompressedRangeData{
      range_data.origin, CompressedPointCloud(range_data.returns),
      CompressedPointCloud(range_data.misses),
  };
}
/*
解压缩，有精度丢失
*/
RangeData Decompress(const CompressedRangeData& compressed_range_data) {
  return RangeData{compressed_range_data.origin,
                   compressed_range_data.returns.Decompress(),
                   compressed_range_data.misses.Decompress()};
}

}  // namespace sensor
}  // namespace cartographer
