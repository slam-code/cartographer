

#include "cartographer/io/points_processor_pipeline_builder.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/io/coloring_points_processor.h"
#include "cartographer/io/counting_points_processor.h"
#include "cartographer/io/fixed_ratio_sampling_points_processor.h"
#include "cartographer/io/intensity_to_color_points_processor.h"
#include "cartographer/io/min_max_range_filtering_points_processor.h"
#include "cartographer/io/null_points_processor.h"
#include "cartographer/io/outlier_removing_points_processor.h"
#include "cartographer/io/pcd_writing_points_processor.h"
#include "cartographer/io/ply_writing_points_processor.h"
#include "cartographer/io/xray_points_processor.h"
#include "cartographer/io/xyz_writing_points_processor.h"
#include "cartographer/mapping/proto/trajectory.pb.h"

namespace cartographer {
namespace io {

/*
注册内置的points Processor处理类,
只有1个参数，类型是PointsProcessorPipelineBuilder。

具体做法是调用Builder的成员函数
Register(const std::string& name, FactoryFunction factory);

第一参数是kConfigurationFileActionName

第二参数是FactoryFunction,即为函数unique_ptr<> f(dictionary,next),
返回值是调用每个Processor的成员函数FromDictionary()
得到unique_ptr<PointsProcessor>.

*/
template <typename PointsProcessorType>
void RegisterPlainPointsProcessor(
    PointsProcessorPipelineBuilder* const builder) 

{
  builder->Register(
      PointsProcessorType::kConfigurationFileActionName,

      [](common::LuaParameterDictionary* const dictionary,
         PointsProcessor* const next) 
      -> std::unique_ptr<PointsProcessor>//返回值是unique_ptr
       {
        return PointsProcessorType::FromDictionary(dictionary, next);
      });
}

/*
RegisterFileWritingPointsProcessor负责注册文件写入的Processor类。
用于将points写入到文件。
第一参数：管理文件写入的FileWriterFactory
第二参数：PointsProcessorPipelineBuilder*

*/
template <typename PointsProcessorType>
void RegisterFileWritingPointsProcessor(
    FileWriterFactory file_writer_factory,
    PointsProcessorPipelineBuilder* const builder)
     {
  builder->Register(
      PointsProcessorType::kConfigurationFileActionName,
      [file_writer_factory](
          common::LuaParameterDictionary* const dictionary,
          PointsProcessor* const next)

           -> std::unique_ptr<PointsProcessor> //返回值是unique_ptr

           {
        return PointsProcessorType::FromDictionary(file_writer_factory,
                                                   dictionary, next);
      });

}

/*.h文件对外的接口,主要的代码逻辑
作用：依次注册所有的内置Processor类。

*/
void RegisterBuiltInPointsProcessors(
    const mapping::proto::Trajectory& trajectory,
    FileWriterFactory file_writer_factory,
    PointsProcessorPipelineBuilder* builder) 

{
  //注册多个非文件相关的Processor实例对象
  RegisterPlainPointsProcessor<CountingPointsProcessor>(builder);
  
  RegisterPlainPointsProcessor<FixedRatioSamplingPointsProcessor>(builder);
  RegisterPlainPointsProcessor<MinMaxRangeFiteringPointsProcessor>(builder);
  RegisterPlainPointsProcessor<OutlierRemovingPointsProcessor>(builder);
  RegisterPlainPointsProcessor<ColoringPointsProcessor>(builder);
  RegisterPlainPointsProcessor<IntensityToColorPointsProcessor>(builder);
    
    //注册多个file Processor实例对象
  RegisterFileWritingPointsProcessor<PcdWritingPointsProcessor>(        
      file_writer_factory, builder);
  RegisterFileWritingPointsProcessor<PlyWritingPointsProcessor>(
      file_writer_factory, builder);
  RegisterFileWritingPointsProcessor<XyzWriterPointsProcessor>(   
      file_writer_factory, builder);

//特殊：轨迹线传引用&id，用于区别不同建图楼层。
  // X-Ray is an odd ball(古怪的，特殊的) since it requires the trajectory to figure out the
  // different building levels we walked on to separate the images.
  builder->Register(
      XRayPointsProcessor::kConfigurationFileActionName,
      [&trajectory, file_writer_factory]

      (
          common::LuaParameterDictionary* const dictionary,
          PointsProcessor* const next
          ) 

      -> std::unique_ptr<PointsProcessor> 

      {
        return XRayPointsProcessor::FromDictionary(
            trajectory, file_writer_factory, dictionary, next);
      });
}


//将用name标识的class添加到hash表中
void PointsProcessorPipelineBuilder::Register(const std::string& name,
                                              FactoryFunction factory) {
  CHECK(factories_.count(name) == 0) << "A points processor named '" << name
                                     << "' has already been registered.";
  factories_[name] = factory;
}

PointsProcessorPipelineBuilder::PointsProcessorPipelineBuilder() {}


/*

1）整个sensor文件夹下最核心的代码。根据.lua文件
用于创建pipeline流水线，登记所有的PointsProcessor类。
返回值pipeline 是points 处理类的集合。内含所有的class集合。

2）pipeline是vector，元素类型是unique_ptr,指向处理point的class。

3）第一个是空指针NullPointsProcessor类,最后调用,用于丢弃所有的points,
其余的按照逆序添加到pipeline中
*/
std::vector<std::unique_ptr<PointsProcessor>>
PointsProcessorPipelineBuilder::CreatePipeline(
    common::LuaParameterDictionary* const dictionary) const {
  std::vector<std::unique_ptr<PointsProcessor>> pipeline;
  // The last consumer in the pipeline must exist, so that the one created after
  // it (and being before it in the pipeline) has a valid 'next' to point to.
  // The last consumer will just drop all points.
  pipeline.emplace_back(common::make_unique<NullPointsProcessor>());

  std::vector<std::unique_ptr<common::LuaParameterDictionary>> configurations =
      dictionary->GetArrayValuesAsDictionaries();

//按action逆序，rbegin()，从尾部添加元素到pipeline中。
  // We construct the pipeline starting at the back.
  for (auto it = configurations.rbegin(); it != configurations.rend(); it++) {

const string action = (*it)->GetString("action");//min_max_range_filter
auto factory_it = factories_.find(action);   
//类型是pair<,>，second是min_max对应的FactoryFunction

    //只有在hash表中的action在可被添加：否则找不到函数实现。
    CHECK(factory_it != factories_.end())
        << "Unknown action '" << action
        << "'. Did you register the correspoinding PointsProcessor?";
    pipeline.push_back(factory_it->second(it->get(), pipeline.back().get()));
/*注意：
上述代码是vector.push_back( f(dict,next));
内部是一个函数，返回unique_ptr,然后插入vector.
vector.back()总是next的Processor。
*/
  }
  return pipeline;
}

}  // namespace io
}  // namespace cartographer

/*

pipeline = {
    {
      action = "min_max_range_filter",--1
      min_range = 1.,
      max_range = 60.,
    },
    {
      action = "dump_num_points",    --2
    },

    -- Gray X-Rays. These only use geometry to color pixels.
    {
      action = "write_xray_image",   --3
      voxel_size = VOXEL_SIZE,
      filename = "xray_yz_all",
      transform = YZ_TRANSFORM,
    },

    ....
}

*/