
#include "cartographer/kalman_filter/unscented_kalman_filter.h"

#include "cartographer/kalman_filter/gaussian_distribution.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace kalman_filter {
namespace {

//返回1*1的矩阵[val]
Eigen::Matrix<double, 1, 1> Scalar(double value) {
  return value * Eigen::Matrix<double, 1, 1>::Identity();
}

//motion运动函数/状态转移函数,g，可以理解为自身的状态变化
// A simple model that copies the first to the second state variable.
Eigen::Matrix<double, 2, 1> g(const Eigen::Matrix<double, 2, 1>& state) {
//值拷贝,返回矩阵[state[0],state[0]],即只修改第二参数
  Eigen::Matrix<double, 2, 1> new_state;
  new_state << state[0], state[0];
  return new_state;
}

//观察/测量矩阵,h: state[0]-5.,可以理解为传感器到landmark的测量距离。
// A simple observation of the first state variable.
Eigen::Matrix<double, 1, 1> h(const Eigen::Matrix<double, 2, 1>& state) {//
  return Scalar(state[0]) - Scalar(5.);
}

TEST(KalmanFilterTest, testConstructor) {
  /*
  构造一个卡尔曼滤波,参数是:
  1.长度为2的高斯分布,均值是0,42,协方差是10*[1,0;0,1]
  2.默认
  3.默认
  */
  UnscentedKalmanFilter<double, 2> filter(GaussianDistribution<double, 2>(
      Eigen::Vector2d(0., 42.), 10. * Eigen::Matrix2d::Identity()));
  EXPECT_NEAR(42., filter.GetBelief().GetMean()[1], 1e-9);
  //均值第1列是42,index 从0开始。
}

TEST(KalmanFilterTest, testPredict) {
//创建一个含2个变量的卡尔曼滤波。均值是42,0,协方差是10*[1,0;0,1]
  UnscentedKalmanFilter<double, 2> filter(GaussianDistribution<double, 2>(
      Eigen::Vector2d(42., 0.), 10. * Eigen::Matrix2d::Identity()));

/*
执行预测step,在预测阶段，滤波器使用上一状态的估计，做出对当前状态的估计。
测量噪声是0均值单位阵。
预测值应该与[42,42]相等：g没有修改第一参数，只修改第二参数使其等于第一参数。
*/
  filter.Predict(
      g, GaussianDistribution<double, 2>(Eigen::Vector2d(0., 0.),
                                         Eigen::Matrix2d::Identity() * 1e-9));
  EXPECT_NEAR(filter.GetBelief().GetMean()[0], 42., 1e-2);
  EXPECT_NEAR(filter.GetBelief().GetMean()[1], 42., 1e-2);
}

TEST(KalmanFilterTest, testObserve) {

//创建一个含2个变量的卡尔曼滤波。均值是0,42,协方差是10*[1,0;0,1]
  UnscentedKalmanFilter<double, 2> filter(GaussianDistribution<double, 2>(
      Eigen::Vector2d(0., 42.), 10. * Eigen::Matrix2d::Identity()));

/*
预测+观察,重复500次,最后均值是[5,5]：
观测矩阵h表示landmark到state[0]的距离永远是5，故稳定后state[0],state[1]应都为5
 
卡尔曼滤波：总是先预测，再根据传感器进行数据融合(校正)。
*/
  for (int i = 0; i < 500; ++i) {
    //预测。motion函数是g，control噪声的均值和方差是0
    filter.Predict(
        g, GaussianDistribution<double, 2>(Eigen::Vector2d(0., 0.),
                                           Eigen::Matrix2d::Identity() * 1e-9));
    //测量/观察。传感器噪声的均值和方差是0
    filter.Observe<1>(
        h, GaussianDistribution<double, 1>(Scalar(0.), Scalar(1e-2)));
  }
  EXPECT_NEAR(filter.GetBelief().GetMean()[0], 5, 1e-2);
  EXPECT_NEAR(filter.GetBelief().GetMean()[1], 5, 1e-2);
}

}  // namespace
}  // namespace kalman_filter
}  // namespace cartographer
/*
 运行unscented_kalman_filter_le_test.cc:

 before:0 42  #滤波前
 Predict:0 0  # 执行预测
 Observe:4.995 4.995 # 执行校正
 --
 before:4.995 4.995
 Predict4.995 4.995
 Observe: 4.9975 4.9975
可见经过2次迭代就已经较为准确了。

*/