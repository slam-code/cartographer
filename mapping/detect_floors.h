
#ifndef CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_
#define CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_

#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {

/*
span,范围
Timespan表征时间范围。
*/
struct Timespan {
  common::Time start;
  common::Time end;
};

/*
一个楼层对应多个扫描timespan：有可能重复的扫描多次
但只有一个高度z。
*/
struct Floor {
// The spans of time we spent on this floor. Since we might have 
//walked up and down many times in this place, there can be many spans
// of time we spent on a particular floor.
  std::vector<Timespan> timespans;

  // The median z-value of this floor.
  double z; //z轴的中值
};

/*
heuristic:启发式,
使用启发式搜索寻找building的不同楼层的z值。
对楼层的要求：同一floor同一z值，只要有“楼梯”出现，即为“产生”一层
*/
// Uses a heuristic which looks at z-values of the poses to detect individual
// floors of a building. This requires that floors are *mostly* on the same
// z-height and that level changes happen *relatively* abrubtly, e.g. by taking
// the stairs.
std::vector<Floor> DetectFloors(const proto::Trajectory& trajectory);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_
