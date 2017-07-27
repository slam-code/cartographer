
#include "cartographer/mapping/probability_values.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

TEST(ProbabilityValuesTest, OddsConversions) {
  EXPECT_NEAR(ProbabilityFromOdds(Odds(kMinProbability)), kMinProbability,
              1e-6);
  EXPECT_NEAR(ProbabilityFromOdds(Odds(kMaxProbability)), kMaxProbability,
              1e-6);
  EXPECT_NEAR(ProbabilityFromOdds(Odds(0.5)), 0.5, 1e-6);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
