
#ifndef CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_
#define CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/data.h"

 
namespace cartographer {
namespace sensor {

struct QueueKey {
  int trajectory_id;// 轨线id;
  string sensor_id; //传感器id

//重载小于运算符,forward_as_tuple:完美转发. (以tuple规则比较2者),tuple定义了<运算符
  bool operator<(const QueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) <
           std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
};


/*

OrderedMultiQueue，用于管理多个有序的传感器数据，
是有序的多队列类,每个队列有一个key,并且有一个自定义排序函数
queues_的形式为：
key1:Queue
key2：Queue
key3：Queue

Queue的形式为
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;
    bool finished = false;
  };

OrderedMultiQueue的数据成员有
1,common_start_time_per_trajectory_:轨迹id及对应创建轨迹时间
2,last_dispatched_time_
3,std::map<QueueKey, Queue> queues_;按照key排序的map
4,QueueKey blocker_;

*/
// Maintains multiple queues of sorted sensor data and dispatches(迅速办理) it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
//
// This class is thread-compatible.
class OrderedMultiQueue {
 public:
  using Callback = std::function<void(std::unique_ptr<Data>)>;//回调函数

  OrderedMultiQueue();
  ~OrderedMultiQueue();


//添加一个【队列】Queue,名称是key,以后入队的data，调用回调函数callback处理
  // Adds a new queue with key 'queue_key' which must not already exist.
  // 'callback' will be called whenever data from this queue can be dispatched.
  void AddQueue(const QueueKey& queue_key, Callback callback);

/*
某一key标识的【队列】Queue已经完成入队,因此不能再入队列,并在map中移除key.
*/
  // Marks a queue as finished, i.e. no further data can be added. The queue
  // will be removed once the last piece of data from it has been dispatched.
  void MarkQueueAsFinished(const QueueKey& queue_key);

//对某一key标识的队列Queue,压入data,data按照回调函数处理
  // Adds 'data' to a queue with the given 'queue_key'. Data must be added
  // sorted per queue.
  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);

/*标记全部队列都已经finished.
 kFirst: {0,4,5,6}
kSecond:{0,1,3,7}
kThird: {0,2,8}
之前只处理到6，调用Flush()则处理剩余的7,8

如果不调用Flush()，则析构时会出错
 */

  // Dispatches all remaining values in sorted order and removes the underlying
  // queues.
  void Flush();

/*返回阻塞的队列(意为该队列对应的sensor的data未到达)*/
  // Must only be called if at least one unfinished queue exists. Returns the
  // key of a queue that needs more data before the OrderedMultiQueue can
  // dispatch data.
  QueueKey GetBlocker() const;

 private:
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;
    bool finished = false;
  };

  void Dispatch();
  void CannotMakeProgress(const QueueKey& queue_key);
  common::Time GetCommonStartTime(int trajectory_id);

  // Used to verify that values are dispatched in sorted order.
  common::Time last_dispatched_time_ = common::Time::min();

  std::map<int, common::Time> common_start_time_per_trajectory_;
  std::map<QueueKey, Queue> queues_;//多队列主体,本类最大的内存占用量
  QueueKey blocker_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_
/*
queues_:
first 是QueueKey
second 是 Queue。

Queue的形式为
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;
    bool finished = false;
  };

*/