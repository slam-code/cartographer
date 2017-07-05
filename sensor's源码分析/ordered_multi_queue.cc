
#include "cartographer/sensor/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace

//重载QueueKey的<<输出运算符,友元函数
inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

OrderedMultiQueue::OrderedMultiQueue() {}

OrderedMultiQueue::~OrderedMultiQueue() {
  for (auto& entry : queues_) {
    CHECK(entry.second.finished);
  }
}

//添加一个关键词是key的队列,并用比较函数Callback排序
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  CHECK_EQ(queues_.count(queue_key), 0);
  //map.count()相对于[],调用时，是不添加key的。而[]是带有副作用的。c++pr.386
  queues_[queue_key].callback = std::move(callback);
}

void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";
  auto& queue = it->second;
  CHECK(!queue.finished);//检查状态
  queue.finished = true;//标记本队列已完成,别的数据不能再入队.
  Dispatch();           //调用一次MarkQueueAsFinished()就要调用一次Dispatch()
}

//根据key找到队列,并添加data元素
void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  auto it = queues_.find(queue_key);
  if (it == queues_.end()) {//没有key时，警告
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }
  it->second.queue.Push(std::move(data));//Queue的queue.push()
  Dispatch();//调用一次Add()就要调用一次Dispatch()
}

//先找到没有finished的队列,然后再对这些队列标记finished.已完成的则不作任何处理
void OrderedMultiQueue::Flush() {
  std::vector<QueueKey> unfinished_queues;
  for (auto& entry : queues_) {
    if (!entry.second.finished) {
      unfinished_queues.push_back(entry.first);
    }
  }
  for (auto& unfinished_queue : unfinished_queues) {
    MarkQueueAsFinished(unfinished_queue);//一个一个的处理。
  }
}

QueueKey OrderedMultiQueue::GetBlocker() const {
  CHECK(!queues_.empty());
  return blocker_;
}

/*
Dispatch()函数，不断的处理来自sensor的数据。按照data采集的时间顺序处理。

kFirst: {1,2,3} finised
kSecond:{}      finised
kThird: {}      finised

*/
void OrderedMultiQueue::Dispatch() {
  while (true) {
    //首先处理的数据，也即最早采集的数据
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;
    //遍历队列中的每一个key：填充上面3个变量值。如果某一key对应的data为空，则直接return
    for (auto it = queues_.begin(); it != queues_.end();) {
      const auto* data = it->second.queue.Peek<Data>();//获取某一队的队首data
      if (data == nullptr) { 
        if (it->second.finished) {//it对应的队列为空且为finished,故删除it对应的key
          queues_.erase(it++);    //map迭代器没有失效?
          continue;
        }
        CannotMakeProgress(it->first);//此时什么也不做。
        return;
      }
      if (next_data == nullptr || data->time < next_data->time) {
      //找到next_data数据:即采集时间最早的数据，理论上应该最先处理它。
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }
      CHECK_LE(last_dispatched_time_, next_data->time)
          << "Non-sorted data added to queue: '" << it->first << "'";
      ++it;
    }


    if (next_data == nullptr) {
      CHECK(queues_.empty());//只有多队列为空，才可能next_data==nullptr
      return;
    }


    // If we haven't dispatched any data for this trajectory yet, fast forward
    // all queues of this trajectory until a common start time has been reached.
    const common::Time common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id);
    //common_start_time即所有的sensor都开始有data的时间点。

    if (next_data->time >= common_start_time) { //大多数情况，happy case
      // Happy case, we are beyond the 'common_start_time' already.
      last_dispatched_time_ = next_data->time;
      next_queue->callback(next_queue->queue.Pop()); //调用回调函数处理data。
    } else if (next_queue->queue.Size() < 2) {// 罕见
      if (!next_queue->finished) {
        // We cannot decide whether to drop or dispatch this yet.
        CannotMakeProgress(next_queue_key);
        return;
      }
      last_dispatched_time_ = next_data->time;
      next_queue->callback(next_queue->queue.Pop());
    } else {
      // We take a peek at the time after next data. If it also is not beyond
      // 'common_start_time' we drop 'next_data', otherwise we just found the
      // first packet to dispatch from this queue.
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->time > common_start_time) {
        last_dispatched_time_ = next_data->time;
        next_queue->callback(std::move(next_data_owner));
      }
    }
  }
}

void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) {
  blocker_ = queue_key; //标识该队列Queue已经阻塞
  for (auto& entry : queues_) {//std::map<QueueKey, Queue> 
    if (entry.second.queue.Size() > kMaxQueueSize) {//？
      LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;
      return;
    }
  }
}

/*对同一轨迹id，求得所有sensor的首次采集data的最晚时间maxtime

不同轨迹按不同轨迹算：
kFirst: {0,1,2,3} finised
kSecond:{2}       
kThird: {4}
{2,2,}

*/
common::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) {
  //map.emplace():Construct and insert element，根据trajectory_id构造一个map。
  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, common::Time::min());
  common::Time& common_start_time = emplace_result.first->second;
  if (emplace_result.second) {
    for (auto& entry : queues_) { 
    //entry是map的pair<,>.本循环求得所有传感器中的maxtime
      if (entry.first.trajectory_id == trajectory_id) {
        common_start_time =
            std::max(common_start_time, entry.second.queue.Peek<Data>()->time);
      }
    }
    LOG(INFO) << "All sensor data for trajectory " << trajectory_id
              << " is available starting at '" << common_start_time << "'.";
  }
  return common_start_time;
}

}  // namespace sensor
}  // namespace cartographer


/*
ordered_multi_queue.cc:172] All sensor data for trajectory 0 is available starting at '635727208200176089'

*/

/*
LOG_EVERY_N(INFO, 60) << "Queue...";在程序中周期性的记录日志信息，
在该语句第1、61、121……次被执行的时候，记录日志信息。
*/