
#ifndef CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#define CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_

#include <cmath>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/*
多个用于计算概率的mapping命名空间下的全局函数

*/
inline float Odds(float probability) {          //论文公式(2),求胜负比。y=x/(1-x)
  return probability / (1.f - probability);
}

inline float ProbabilityFromOdds(const float odds) { //求概率，即x=y/(1+y)
  return odds / (odds + 1.f);
}

constexpr float kMinProbability = 0.1f;//p最小是0.1
constexpr float kMaxProbability = 1.f - kMinProbability;//最大是0.9

/*
限制概率p在[0.1,0.9]之间
*/
// Clamps probability to be in the range [kMinProbability, kMaxProbability].
inline float ClampProbability(const float probability) {
  return common::Clamp(probability, kMinProbability, kMaxProbability);
}

constexpr uint16 kUnknownProbabilityValue = 0;//标记未初始化的概率
constexpr uint16 kUpdateMarker = 1u << 15;// 32768

/*
将概率p映射为整数Value，
[0.0,1.0]:
->[0.1,0.9]
->[1,32767]
*/
// Converts a probability to a uint16 in the [1, 32767] range.
inline uint16 ProbabilityToValue(const float probability) {
  const int value =
      common::RoundToInt((ClampProbability(probability) - kMinProbability) *
                         (32766.f / (kMaxProbability - kMinProbability))) +
      1;
  // DCHECK for performance.
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

//声明，定义在.cc文件。vector是value到p的映射
extern const std::vector<float>* const kValueToProbability;


/*
反映射 [1,32767]->[0.1,0.9]
利用查表法，快速获得值，而不是用math函数求值。
*/
// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
inline float ValueToProbability(const uint16 value) {
  return (*kValueToProbability)[value];
}

//2份value:前一半对应没有hit，后一半对应hit。
std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_

/*

Odds()含义:
Odds are ratios of a player’s chances of losing to his or her chances of winning, or the average frequency of a loss to the average frequency of a win. 
http://www.problemgambling.ca/en/resourcesforprofessionals/pages/probabilityoddsandrandomchance.aspx


*/