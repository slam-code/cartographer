#ifndef CARTOGRAPHER_MAPPING_ID_H_
#define CARTOGRAPHER_MAPPING_ID_H_

#include <algorithm>
#include <ostream>
#include <tuple>
#include <vector>

namespace cartographer {
namespace mapping {

/*
每一个轨迹trajectory上有多个节点node。
如何唯一的表达这些节点node？节点标号:轨迹id+{0,1,2,...}
*/
// Uniquely identifies a trajectory node using a combination of a unique
// trajectory ID and a zero-based index of the node inside that trajectory.
struct NodeId {
  int trajectory_id;
  int node_index;

//比较<符号，成员函数
  bool operator<(const NodeId& other) const {
    return std::forward_as_tuple(trajectory_id, node_index) <
           std::forward_as_tuple(other.trajectory_id, other.node_index);
  }
};

//<<运算符，友元函数
inline std::ostream& operator<<(std::ostream& os, const NodeId& v) {
  return os << "(" << v.trajectory_id << ", " << v.node_index << ")";
}

/*
一般来说，重建全局地图global map时，是由多个submap组成。
如何给这些submap标号? 轨迹id+ {0,1,2,3...}
*/
// Uniquely identifies a submap using a combination of a unique trajectory ID
// and a zero-based index of the submap inside that trajectory.
struct SubmapId {
  int trajectory_id;
  int submap_index;

  bool operator<(const SubmapId& other) const {
    return std::forward_as_tuple(trajectory_id, submap_index) <
           std::forward_as_tuple(other.trajectory_id, other.submap_index);
  }
};

//友元函数
inline std::ostream& operator<<(std::ostream& os, const SubmapId& v) {
  return os << "(" << v.trajectory_id << ", " << v.submap_index << ")";
}

/*
Nested,嵌套.
NestedVectorsById类只有1个数据成员,vector<std::vector<ValueType>> data_;

作用：嵌套存储多条轨迹线上的数据
对外提供3个操作
1,Append()根据轨迹id和data添加数据
2,at()查询某轨迹对应的vector的某值
3,num_indices(),某轨迹对应的数据的大小
*/
template <typename ValueType, typename IdType>
class NestedVectorsById {
 public:

/*
data_是vector,size()是轨迹的数量。data_存储的是轨迹id
data_[i]也是vector.存储的是轨迹上的data
向data_[i]添加数据。
*/
  // Appends data to a trajectory, creating trajectories as needed.
  IdType Append(int trajectory_id, const ValueType& value) {
    data_.resize(std::max<size_t>(data_.size(), trajectory_id + 1));

  //构造一个IdType对象用于返回，该id是轨迹id+{n}
    const IdType id{trajectory_id,
                    static_cast<int>(data_[trajectory_id].size())};
    data_[trajectory_id].push_back(value);
    return id;
  }

//在某一轨迹id上对应的data
  const ValueType& at(const IdType& id) const {
    //vector的成员函数:at(size_type n);即v[x][y]
    return data_.at(id.trajectory_id).at(GetIndex(id));
  }
  ValueType& at(const IdType& id) {
    return data_.at(id.trajectory_id).at(GetIndex(id));
  }

  //轨迹数量data_的size()
  int num_trajectories() const { return static_cast<int>(data_.size()); }

  //某个轨迹对应的的大小,data_[i].size()
  int num_indices(int trajectory_id) const {
    return static_cast<int>(data_.at(trajectory_id).size());
  }

  // TODO(whess): Remove once no longer needed.
  const std::vector<std::vector<ValueType>> data() const { return data_; }

 private:
  //返回id对应的index，即{0,1,2,...}
  static int GetIndex(const NodeId& id) { return id.node_index; }
  static int GetIndex(const SubmapId& id) { return id.submap_index; }

  std::vector<std::vector<ValueType>> data_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_ID_H_
