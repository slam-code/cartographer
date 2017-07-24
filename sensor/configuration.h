
#ifndef CARTOGRAPHER_SENSOR_CONFIGURATION_H_
#define CARTOGRAPHER_SENSOR_CONFIGURATION_H_

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/sensor/proto/configuration.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {

//从sensor配置文件解析sensor的数据参数。主要是sensor到机器坐标的转换
// Creates the configuration for a singular sensor from 'parameter_dictionary'.
proto::Configuration::Sensor CreateSensorConfiguration(
    common::LuaParameterDictionary* parameter_dictionary);

//求得多个sensor的配置的集合。
// Creates the mapping from frame_id to Sensors defined in
// 'parameter_dictionary'.
proto::Configuration CreateConfiguration(
    common::LuaParameterDictionary* parameter_dictionary);

//系统是否支持某一传感器。
// Returns true if 'frame_id' is mentioned in 'sensor_configuration'.
bool IsEnabled(const string& frame_id,
               const sensor::proto::Configuration& sensor_configuration);

//将sensor采集的data经过3d坐标变换为机器坐标。
// Returns the transform which takes data from the sensor frame to the
// tracking frame.
transform::Rigid3d GetTransformToTracking(
    const string& frame_id,
    const sensor::proto::Configuration& sensor_configuration);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_CONFIGURATION_H_
