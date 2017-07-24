
#include "cartographer/sensor/voxel_filter.h"

#include <cmath>

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::ContainerEq;

TEST(VoxelFilterTest, ReturnsTheFirstPointInEachVoxel) {
  PointCloud point_cloud = {{0.f, 0.f, 0.f},
                            {0.1f, -0.1f, 0.1f},
                            {0.3f, -0.1f, 0.f},
                            {0.f, 0.f, 0.1f}};
  EXPECT_THAT(VoxelFiltered(point_cloud, 0.3f),//按照max_range滤波,
              ContainerEq(PointCloud{point_cloud[0], point_cloud[2]}));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer

/*
ContainerEq:
The same as Eq(container) except that the failure message also 
includes which elements are in one container but not the other.

*/