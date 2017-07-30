
#ifndef CARTOGRAPHER_MAPPING_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {


//求odds(p)的log对数
// Converts the given probability to log odds.
inline float Logit(float probability) {
  return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);


/*
将[0.1,0.9]映射为0-255之间的数
*/
// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
inline uint8 ProbabilityToLogOddsInteger(const float probability) {
  const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                       254.f / (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

/*
一个独立的子图,在局部slam frame中有一个local_pose,
追踪有多少range data 添加到其中.并设置finished_probability_grid用于闭环检查

数据成员有
1,local_pose                 位姿
2,num_range_data             已经插入的测量数据数量
3,finished_probability_grid  完成建图的概率网格

*/

// An individual submap, which has a 'local_pose' in the local SLAM frame, keeps
// track of how many range data were inserted into it, and sets the
// 'finished_probability_grid' to be used for loop closing once the map no
// longer changes.
struct Submap {
  Submap(const transform::Rigid3d& local_pose) : local_pose(local_pose) {}

  // Local SLAM pose of this submap.
  const transform::Rigid3d local_pose;//子图的位姿

  // Number of RangeData inserted.
  int num_range_data = 0;

  // The 'finished_probability_grid' when this submap is finished and will not
  // change anymore. Otherwise, this is nullptr and the next call to
  // InsertRangeData() will change the submap.
  //当子图不再变化时，指向该子图的概率分布网格。
  const mapping_2d::ProbabilityGrid* finished_probability_grid = nullptr;
};

/*

 Submaps is a sequence of maps to which scans are matched and into which scans are inserted.
 Except during initialization when only a single submap exists, there are
 always two submaps into which scans are inserted: an old submap that is used
 for matching, and a new one, which will be used for matching next, that is
 being initialized.

 Once a certain number of scans have been inserted, the new submap is
 considered initialized: the old submap is no longer changed, the "new" submap
 is now the "old" submap and is used for scan-to-map matching. Moreover,
 a "new" submap gets inserted.

Submaps是一连串的子图,初始化以后任何阶段均有2个子图被当前scan point影响：
old submap 用于now match,new submap用于next match,一直交替下去。
一旦new submap有足够多的scan point，那么old submap不再更新。
此时new submap变为old，用于 scan-to-map匹配。

Submaps不可拷贝/赋值,是虚基类

虚基类,没有数据成员,只提供成员函数用于实现接口。

成员函数 
1,matching_index()。返回最后一个submap的索引,用于配准:len-2
2,insertion_indices()。返回最后2个submap的索引,用于点云插入;{len-2,len-1}
3,Get()返回给定索引的子图
4,AddProbabilityGridToResponse()
*/
class Submaps {
 public:
  static constexpr uint8 kUnknownLogOdds = 0;

  Submaps();
  virtual ~Submaps();

  Submaps(const Submaps&) = delete;
  Submaps& operator=(const Submaps&) = delete;

  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  // size() - 2;
  int matching_index() const;//最新的初始化的子图索引，用于scan-to-map-match

  // Returns the indices of the Submap into which point clouds will
  // be inserted.
  // {size() - 2, size() - 1};
  std::vector<int> insertion_indices() const;//待插入的子图的索引

  // Returns the Submap with the given 'index'. The same 'index' will always
  // return the same pointer, so that Submaps can be identified by it.
  virtual const Submap* Get(int index) const = 0;//纯虚函数，按索引返回子图指针。

  // Returns the number of Submaps.
  virtual int size() const = 0;

  //将对应的子图序列化到proto文件中
  // Fills data about the Submap with 'index' into the 'response'.
  virtual void SubmapToProto(int index,
                             const transform::Rigid3d& global_submap_pose,
                             proto::SubmapQuery::Response* response) const = 0;

 protected:
  //将子图对应的概率网格序列化到proto文件中
  static void AddProbabilityGridToResponse(
      const transform::Rigid3d& local_submap_pose,
      const mapping_2d::ProbabilityGrid& probability_grid,
      proto::SubmapQuery::Response* response);
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SUBMAPS_H_
