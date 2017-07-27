
#include "cartographer/sensor/range_data.h"

#include <utility>
#include <vector>

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::Contains;
using ::testing::PrintToString;

// Custom matcher for Eigen::Vector3f entries.
MATCHER_P(ApproximatelyEquals, expected,
          string("is equal to ") + PrintToString(expected)) {
  return (arg - expected).isZero(0.001f);
}

TEST(RangeDataTest, Compression) {
  const std::vector<Eigen::Vector3f> returns = {Eigen::Vector3f(0, 1, 2),
                                                Eigen::Vector3f(4, 5, 6),
                                                Eigen::Vector3f(0, 1, 2)};
   //构造一个测量数据
  const RangeData range_data = {
      Eigen::Vector3f(1, 1, 1), returns, {Eigen::Vector3f(7, 8, 9)}};
  //压缩再解压，减少空间占用。
  const RangeData actual = Decompress(Compress(range_data));

//isApprox():true if *this is approximately equal to other, within the precision determined by prec.
 //压缩前后，精度比较
  EXPECT_TRUE(actual.origin.isApprox(Eigen::Vector3f(1, 1, 1), 1e-6));


  EXPECT_EQ(3, actual.returns.size());

  EXPECT_EQ(1, actual.misses.size());

  EXPECT_TRUE(actual.misses[0].isApprox(Eigen::Vector3f(7, 8, 9), 0.001f));

  // Returns will be reordered, so we compare in an unordered manner.
  EXPECT_EQ(3, actual.returns.size());

  //是否包含给定值，returns是乱序的。所以使用Contains()函数。
  EXPECT_THAT(actual.returns,
              Contains(ApproximatelyEquals(Eigen::Vector3f(0, 1, 2))));
  EXPECT_THAT(actual.returns,
              Contains(ApproximatelyEquals(Eigen::Vector3f(4, 5, 6))));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
