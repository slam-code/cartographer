
#ifndef CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_
#define CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_

#include "Eigen/Cholesky"
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace cartographer {
namespace kalman_filter {
/*
高斯分布类
构造函数是N*1的均值矩阵和N*N的协方差矩阵

*/
template <typename T, int N>
class GaussianDistribution {
 public:
  GaussianDistribution(const Eigen::Matrix<T, N, 1>& mean,
                       const Eigen::Matrix<T, N, N>& covariance)
      : mean_(mean), covariance_(covariance) {}

  const Eigen::Matrix<T, N, 1>& GetMean() const { return mean_; }

  const Eigen::Matrix<T, N, N>& GetCovariance() const { return covariance_; }

 private:
  Eigen::Matrix<T, N, 1> mean_;       //N*1，均值
  Eigen::Matrix<T, N, N> covariance_; //N*N。协方差
};

/*
重载+加号操作符,
高斯+高斯=对应均值+均值,对应协方差+协方差
返回值：新高斯对象
高斯分布性质:
https://zh.wikipedia.org/wiki/%E6%AD%A3%E6%80%81%E5%88%86%E5%B8%83
*/
template <typename T, int N>
GaussianDistribution<T, N> operator+(const GaussianDistribution<T, N>& lhs,
                                     const GaussianDistribution<T, N>& rhs) {
  return GaussianDistribution<T, N>(lhs.GetMean() + rhs.GetMean(),
                                    lhs.GetCovariance() + rhs.GetCovariance());
}

/*
重载乘法运算符
1,矩阵N*M
2,高斯分布M*M

返回值:高斯分布：N*N


*/
template <typename T, int N, int M>
GaussianDistribution<T, N> operator*(const Eigen::Matrix<T, N, M>& lhs,
                                     const GaussianDistribution<T, M>& rhs) {
  return GaussianDistribution<T, N>(
      lhs * rhs.GetMean(),                          // N*M || M*1 -> N*1

      lhs * rhs.GetCovariance() * lhs.transpose()); // N*M ||M*M || M*N ->  N*N
}

}  // namespace kalman_filter
}  // namespace cartographer

#endif  // CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_
