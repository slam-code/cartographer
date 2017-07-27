
#ifndef CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
#define CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace transform {

/*
基于时间有序的变换,支持在队列中按时间顺序查找,即使变换不存在于队列中，任然支持相邻时间内的插值变换进行近似。
作用与ROS的tf2函数族类似。

数据成员：
1,deque_;
成员函数：
1,Push()
2,Has()
3,Lookup()
4,earliest_time()
5,latest_time()
6,empty()
*/
// A time-ordered buffer of transforms that supports interpolated lookups.
class TransformInterpolationBuffer {
 public:
 	//函数,返回智能指针
  static std::unique_ptr<TransformInterpolationBuffer> FromTrajectory(
      const mapping::proto::Trajectory& trajectory);

/*
添加变换到队列尾部,当缓冲区已满时,删除队首元素
*/
  // Adds a new transform to the buffer and removes the oldest transform if the
  // buffer size limit is exceeded.
  void Push(common::Time time, const transform::Rigid3d& transform);

//返回能否在给定时间内计算的插值变换。time应在early-old之间，可以插值。
  // Returns true if an interpolated transfrom can be computed at 'time'
  bool Has(common::Time time) const;
//返回time处的变换,可插值
  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  transform::Rigid3d Lookup(common::Time time) const;

/*
返回队列缓冲区内变换的最早时间，也就是队首元素。
*/
  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time earliest_time() const;
/*
最晚时间，也就是队尾元素
*/
  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time latest_time() const;

  // Returns true if the buffer is empty.
  bool empty() const;

 private:
  struct TimestampedTransform {
    common::Time time;           //发生时间
    transform::Rigid3d transform;//变换矩阵
  };

  std::deque<TimestampedTransform> deque_;
  //队列，元素是带时间戳的变换,存储了一段时间内的变换矩阵信息
};

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
