

#include "cartographer/sensor/collator.h"

namespace cartographer {
namespace sensor {

/*
一个轨迹线,多个传感器

*/

/*
struct QueueKey {
  int trajectory_id;// 轨线id;
  string sensor_id; //传感器id
  }

*/
void Collator::AddTrajectory(
    const int trajectory_id,
    const std::unordered_set<string>& expected_sensor_ids,
    const Callback callback) {
  for (const auto& sensor_id : expected_sensor_ids) { 
    //对于每一个轨迹线+传感器,设置一个key
    const auto queue_key = QueueKey{trajectory_id, sensor_id};

    //添加一个名为key的队列,并设置回调函数处理data
    queue_.AddQueue(queue_key,
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });

    //map<int,vector<key>>:添加轨迹线对应的key
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}


/*队列不再接收数据*/
void Collator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    queue_.MarkQueueAsFinished(queue_key);
  }
}


/*
主要的操作,添加传感器数据,数据形式是:key+data
*/
void Collator::AddSensorData(const int trajectory_id, const string& sensor_id,
                             std::unique_ptr<Data> data) {
  //找到key，再move(data)
  queue_.Add(QueueKey{trajectory_id, sensor_id}, std::move(data));
}

void Collator::Flush() { queue_.Flush(); }

int Collator::GetBlockingTrajectoryId() const {
  return queue_.GetBlocker().trajectory_id;
}

}  // namespace sensor
}  // namespace cartographer
