
#include "cartographer/sensor/configuration.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/sensor/proto/configuration.pb.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {
/*
assets_writer_backpack_2d.lua

Sensor定义：
  message Sensor {
  string frame_id = 2; //'frame_id' of the sensor.
  Rigid3d transform = 3;
  }
*/
proto::Configuration::Sensor CreateSensorConfiguration(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::Configuration::Sensor sensor;
  sensor.set_frame_id(parameter_dictionary->GetString("frame_id"));

  *sensor.mutable_transform() = transform::ToProto(transform::FromDictionary(
      parameter_dictionary->GetDictionary("transform").get()));
  return sensor;
}

/*
Configuration定义：
message Configuration {
    repeated Sensor sensor = 15;
}
*/
proto::Configuration CreateConfiguration(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::Configuration configuration;
  for (auto& sensor_parameter_dictionary :
       parameter_dictionary->GetArrayValuesAsDictionaries()) {
    *configuration.add_sensor() =
        CreateSensorConfiguration(sensor_parameter_dictionary.get());
  }
  return configuration;
}

/*
判断系统是否支持某一传感器id。
*/
bool IsEnabled(const string& frame_id,
               const sensor::proto::Configuration& sensor_configuration) {
  for (const auto& sensor : sensor_configuration.sensor()) {
    if (sensor.frame_id() == frame_id) {
      return true;
    }
  }
  return false;
}

/*

将sensor坐标转换为机器坐标。以便于跟踪计算
*/
transform::Rigid3d GetTransformToTracking(
    const string& frame_id,
    const sensor::proto::Configuration& sensor_configuration) {
  for (const auto& sensor : sensor_configuration.sensor()) {
    if (sensor.frame_id() == frame_id) {
      return transform::ToRigid3(sensor.transform());
    }
  }
  LOG(FATAL) << "No configuration found for sensor with frame ID '" << frame_id
             << "'.";
}

}  // namespace sensor
}  // namespace cartographer
