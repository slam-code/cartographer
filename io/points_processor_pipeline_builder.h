

#ifndef CARTOGRAPHER_IO_POINTS_PROCESSOR_PIPELINE_BUILDER_H_
#define CARTOGRAPHER_IO_POINTS_PROCESSOR_PIPELINE_BUILDER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/proto/trajectory.pb.h"

namespace cartographer {
namespace io {

/*
使用Builder模式创建PointsProcessor对象：

PointsProcessorPipelineBuilder类用于创建各个ProcessorPiont类对象

调用RegisterBuiltInPointsProcessors()
函数可用于创建所有cartographer内置的PointsProcessor类.
*/

// Builder to create a points processor pipeline out of a Lua configuration.
// You can register all built-in PointsProcessors using
// 'RegisterBuiltInPointsProcessors'. Non-built-in PointsProcessors must define
// a name and a factory method for building itself from a
// LuaParameterDictionary. See the various built-in PointsProcessors for
// examples.
class PointsProcessorPipelineBuilder {
 public:

  //FactoryFunction是函数f(dictionary,next); 
  //返回值为std::unique_ptr<PointsProcessor>
  using FactoryFunction = std::function<std::unique_ptr<PointsProcessor> (
      common::LuaParameterDictionary*, PointsProcessor* next) >;

  PointsProcessorPipelineBuilder();

  PointsProcessorPipelineBuilder(const PointsProcessorPipelineBuilder&) =
      delete;
  PointsProcessorPipelineBuilder& operator=(
      const PointsProcessorPipelineBuilder&) = delete;

  // Register a new PointsProcessor type uniquly identified by 'name' which will
  // be created using 'factory'.
  void Register(const std::string& name, FactoryFunction factory);

  std::vector<std::unique_ptr<PointsProcessor>> CreatePipeline(
      common::LuaParameterDictionary* dictionary) const;

 private:

  //hash表,记录已有代码实现的points processor及其对应的function
  //key是每个class都有的name。
  std::unordered_map<std::string, FactoryFunction> factories_; 

};

// Register all 'PointsProcessor' that ship with Cartographer with this
// 'builder'.
void RegisterBuiltInPointsProcessors(
    const mapping::proto::Trajectory& trajectory,
    FileWriterFactory file_writer_factory,
    PointsProcessorPipelineBuilder* builder);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_POINTS_PROCESSOR_PIPELINE_BUILDER_H_
