
#define sout(Xit)  {std::cout<<__LINE__<<" "<< Xit <<""<<std::endl;}

#include "cartographer/sensor/ordered_multi_queue.h"

#include <vector>

#include "cartographer/common/make_unique.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {
/*测试夹具（Test Fixtures）
TEST_F提供了一个初始化函数（SetUp）和一个清理函数(TearDown)，
在TEST_F中使用的变量可以在初始化函数SetUp中初始化，
在TearDown中销毁，并且所有的TEST_F是互相独立的，都是在初始化以后的状态开始运行，
一个TEST_F不会影响另一个TEST_F所使用的数据.
用TEST_F定义测试，写法与TEST相同，但测试用例名必须为上面定义的类名。
*/
class OrderedMultiQueueTest : public ::testing::Test {
 protected:
  // These are keys are chosen so that they sort first, second, third.
  const QueueKey kFirst{1, "a"};//trajectory_id，sensor_id
  const QueueKey kSecond{1, "b"};
  const QueueKey kThird{2, "b"};

//void AddQueue(const QueueKey& queue_key, Callback callback);
  void SetUp() {
  //初始化函数,定义处理data的函数：
  //(以后用于test比较):检查values_的最大值是否小于欲添加的值.
    for (const auto& queue_key : {kFirst, kSecond, kThird}) {
      queue_.AddQueue(queue_key, [this](std::unique_ptr<Data> data) {
        if (!values_.empty()) {
          EXPECT_GE(data->time, values_.back().time);
        }
        values_.push_back(*data);
      });
    }
  }

//Data的定义在data.h
  std::unique_ptr<Data> MakeImu(const int ordinal) {//时间,序列
    return common::make_unique<Data>(
        common::FromUniversal(ordinal),
        Data::Imu{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
  }

  std::vector<Data> values_;
  OrderedMultiQueue queue_;
};


TEST_F(OrderedMultiQueueTest, Ordering) {
  // void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);
  queue_.Add(kFirst, MakeImu(0));//
  queue_.Add(kFirst, MakeImu(4));
  queue_.Add(kFirst, MakeImu(5));
  queue_.Add(kFirst, MakeImu(6));
  EXPECT_TRUE(values_.empty());  
  //why为空?：queue_对应的其他sensor还没有产生data，
  //故调用Add()时，只在本队列插入data，而Dispatch()直接返回，没有调用Callback函数。


  queue_.Add(kSecond, MakeImu(0));
  queue_.Add(kSecond, MakeImu(1));
  EXPECT_TRUE(values_.empty()); //同上


  queue_.Add(kThird, MakeImu(0));
  queue_.Add(kThird, MakeImu(2));
  EXPECT_EQ(values_.size(), 4);

 /*
kFirst: {0,4,5,6}
kSecond:{0,1}
kThird: {0,2}

*/
  for(auto i:values_)
  {
      sout(i.time);//{0,0,0,1}
  }


  queue_.Add(kSecond, MakeImu(3));
  EXPECT_EQ(values_.size(), 5);
/*
kFirst: {0,4,5,6}
kSecond:{0,1,3}
kThird: {0,2}

*/
  for(auto i:values_)
  {
      sout(i.time);//{0,0,0,1,2}
  }

  queue_.Add(kSecond, MakeImu(7));
  queue_.Add(kThird, MakeImu(8));

  /*
kFirst: {0,4,5,6}
kSecond:{0,1,3,7}
kThird: {0,2,8}

*/
  for(auto i:values_)
  {
      sout(i.time);//{0,0,0,1,2,3,4,5,6}
  }


//先找到没有finished的队列,然后再对这些队列标记finished
  queue_.Flush();//应该调用,否则析构时出错
  EXPECT_EQ(11, values_.size());// 4+4+3

    /*
kFirst: {0,4,5,6}
kSecond:{0,1,3,7}
kThird: {0,2,8}

*/
  for(auto i:values_)
  {
      sout(i.time);//{0,0,0,1,2,3,4,5,6,7,8}
  }
  for (size_t i = 0; i < values_.size() - 1; ++i) { //检查是否按序
    EXPECT_LE(values_[i].time, values_[i + 1].time);
  }
}

TEST_F(OrderedMultiQueueTest, MarkQueueAsFinished) {
  queue_.Add(kFirst, MakeImu(1));
  queue_.Add(kFirst, MakeImu(2));
  queue_.Add(kFirst, MakeImu(3));
  EXPECT_TRUE(values_.empty());
  queue_.MarkQueueAsFinished(kFirst);

//调用该函数一次，就调用一次Dispatch(),若key对应的队列为空，则从队列中删除key
  EXPECT_TRUE(values_.empty());
  queue_.MarkQueueAsFinished(kSecond);
  EXPECT_TRUE(values_.empty());
  queue_.MarkQueueAsFinished(kThird);

    /*
kFirst: {1,2,3} finised
kSecond:{}      finised
kThird: {}      finised

*/
  for(auto i:values_)
  {
      sout(i.time);//{1,2,3}
  }

  EXPECT_EQ(3, values_.size());
  for (size_t i = 0; i < values_.size(); ++i) {
    EXPECT_EQ(i + 1, common::ToUniversal(values_[i].time));
  }
}

TEST_F(OrderedMultiQueueTest, CommonStartTimePerTrajectory) {
  queue_.Add(kFirst, MakeImu(0));
  queue_.Add(kFirst, MakeImu(1));
  queue_.Add(kFirst, MakeImu(2));
  queue_.Add(kFirst, MakeImu(3));
  queue_.Add(kSecond, MakeImu(2));
  EXPECT_TRUE(values_.empty());
  queue_.Add(kThird, MakeImu(4));
  EXPECT_EQ(values_.size(), 2);
/*
kFirst: {0,1,2,3}
kSecond:{2}
kThird: {4}
*/
  for(auto i:values_)
  {
      sout(i.time);//{2,2}
  }


  queue_.MarkQueueAsFinished(kFirst);
  EXPECT_EQ(values_.size(), 2);//没有0,1，因为起点时间不是0,1，而是2
/*
kFirst: {0,1,2,3} finised
kSecond:{2}       
kThird: {4}
*/
  for(auto i:values_)
  {
      sout(i.time);//{2,2}
  }

  sout(".id2.");//为何与test1不同？起点time不同。test1：起点时间全为0，test3起点时间是2
  queue_.MarkQueueAsFinished(kSecond);
  EXPECT_EQ(values_.size(), 4);
/*
kFirst: {0,1,2,3} finised
kSecond:{2}       finised
kThird: {4}
*/
  for(auto i:values_)
  {
      sout(i.time);//{2,2,3,4}
  }

//第二条轨迹id开始采集数据。
  queue_.MarkQueueAsFinished(kThird);
  EXPECT_EQ(values_.size(), 4);
/*
kFirst: {0,1,2,3} finised
kSecond:{2}       finised
kThird: {4}       finised
*/
  for(auto i:values_)
  {
      sout(i.time);//{2,2,3,4}
  }
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
