

#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"

#include <cmath>

#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

TEST(PointCloudTest, TransformPointCloud) {
  PointCloud point_cloud;
  point_cloud.emplace_back(0.5f, 0.5f, 1.f);  //构造一个点云{0.5,0.5,1}
  point_cloud.emplace_back(3.5f, 0.5f, 42.f); //{3.5,0.5,42}

  //调用static Rigid2 Rotation(const double rotation) 
  const PointCloud transformed_point_cloud = TransformPointCloud(
      point_cloud, transform::Embed3D(transform::Rigid2f::Rotation(M_PI_2))); 

/*绕z轴逆时针旋转 pi/2:
[x',y',x']=[]*[x,y,z]
化简后： x=-y，y=x，z=z
*/
  EXPECT_NEAR(-0.5f, transformed_point_cloud[0].x(), 1e-6);
  EXPECT_NEAR(0.5f, transformed_point_cloud[0].y(), 1e-6);
  EXPECT_NEAR(-0.5f, transformed_point_cloud[1].x(), 1e-6);
  EXPECT_NEAR(3.5f, transformed_point_cloud[1].y(), 1e-6);
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
