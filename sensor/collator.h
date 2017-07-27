

#ifndef CARTOGRAPHER_SENSOR_COLLATOR_H_
#define CARTOGRAPHER_SENSOR_COLLATOR_H_

#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cartographer/sensor/data.h"
#include "cartographer/sensor/ordered_multi_queue.h"

namespace cartographer {
namespace sensor {

/*
Collator,采集者，校对者，整理者
将多传感器采集的数据归并到轨迹上。

Collator类:不可拷贝,不可赋值.
只有一个默认构造函数.
有2个数据成员

1,OrderedMultiQueue queue_; key是pair<轨迹线id,传感器id>.
一般情况下，对于已有的bag文件，轨迹id等于0.

2,std::unordered_map<int, std::vector<QueueKey>> queue_keys_
轨迹线和队列key组成的hash表,1：N模式



Collator类主要提供三个操作:
1,AddTrajectory() 添加一个轨迹线,
2,FinishTrajectory() 标记轨迹线已经采集完成
3,AddSensorData()接收传感器数据
4,Flush()刷新

*/
class Collator {
 public:
  using Callback = std::function<void(const string&, std::unique_ptr<Data>)>;

  Collator() {}

  Collator(const Collator&) = delete;
  Collator& operator=(const Collator&) = delete;

/*添加一个轨迹线,接受有序的传感器数据,并使用callback回调处理data。
  一个轨迹线对应多个传感器数据:id ->unordered_set
*/
  // Adds a trajectory to produce sorted sensor output for. Calls 'callback'
  // for each collated sensor data.
  void AddTrajectory(int trajectory_id,
                     const std::unordered_set<string>& expected_sensor_ids,
                     Callback callback);

  // Marks 'trajectory_id' as finished.标记轨迹线已经完成采样.
  void FinishTrajectory(int trajectory_id);

/*添加一个传感器id对应的数据data,数据必须按时间排序，*/
  // Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
  // sensor data. Sensor packets with matching 'sensor_id' must be added in time
  // order.
  void AddSensorData(int trajectory_id, const string& sensor_id,
                     std::unique_ptr<Data> data);


//刷新
  // Dispatches all queued sensor packets. May only be called once.
  // AddSensorData may not be called after Flush.
  void Flush();


/*返回阻塞的轨迹id,
此种情况多见于某一传感器持久未采集data，
造成ordered_multi_queue阻塞。

  */
  // Must only be called if at least one unfinished trajectory exists. Returns
  // the ID of the trajectory that needs more data before the Collator is
  // unblocked.
  int GetBlockingTrajectoryId() const;

 private:
  //多个key构成的多队列
  // Queue keys are a pair of trajectory ID and sensor identifier.
  OrderedMultiQueue queue_;
  
  //int为传感器id,vector是id+sensor组成的QueueKey
  
  // Map of trajectory ID to all associated QueueKeys.
  std::unordered_map<int, std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COLLATOR_H_
