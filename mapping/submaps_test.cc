
#include "cartographer/mapping/submaps.h"

#include <cmath>

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

// Converts the given log odds to a probability. This function is known to be
// very slow, because expf is incredibly slow.
inline float Expit(float log_odds) {  //求指数倍
  const float exp_log_odds = std::exp(log_odds);
  return exp_log_odds / (1.f + exp_log_odds);
}

TEST(SubmapsTest, LogOddsConversions) {
  EXPECT_NEAR(Expit(Logit(kMinProbability)), kMinProbability, 1e-6);
  EXPECT_NEAR(Expit(Logit(kMaxProbability)), kMaxProbability, 1e-6);
  EXPECT_NEAR(Expit(Logit(0.5)), 0.5, 1e-6); //变换再反变换- >不变
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
