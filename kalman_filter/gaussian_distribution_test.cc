#include "cartographer/kalman_filter/gaussian_distribution.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace kalman_filter {
namespace {

TEST(GaussianDistributionTest, testConstructor) {
  Eigen::Matrix2d covariance;//2*2
  covariance << 1., 2., 3., 4.;
  GaussianDistribution<double, 2> distribution(Eigen::Vector2d(0., 1.),
  //均值是0,1
                                               covariance);
  EXPECT_NEAR(0., distribution.GetMean()[0], 1e-9);//0列的的均值是0
  EXPECT_NEAR(1., distribution.GetMean()[1], 1e-9);
  EXPECT_NEAR(2., distribution.GetCovariance()(0, 1), 1e-9);//0行1列的方差是2
}

}  // namespace
}  // namespace kalman_filter
}  // namespace cartographer
