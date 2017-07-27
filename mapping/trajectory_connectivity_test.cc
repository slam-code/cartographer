
#include "cartographer/mapping/trajectory_connectivity.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr int kNumTrajectories = 10;//轨迹数

TEST(TrajectoryConnectivityTest, TransitivelyConnected) {
  TrajectoryConnectivity trajectory_connectivity;

  // Make sure nothing's connected until we connect some things.
  for (int trajectory_a = 0; trajectory_a < kNumTrajectories; ++trajectory_a) {
    for (int trajectory_b = 0; trajectory_b < kNumTrajectories;
         ++trajectory_b) {
      //默认不连通
      EXPECT_FALSE(trajectory_connectivity.TransitivelyConnected(trajectory_a,
                                                                 trajectory_b));
    }
  }

  // Connect some stuff up.
  trajectory_connectivity.Connect(0, 1);                            //联通0-1
  EXPECT_TRUE(trajectory_connectivity.TransitivelyConnected(0, 1)); //true 
  trajectory_connectivity.Connect(8, 9);                            //联通8-9
  EXPECT_TRUE(trajectory_connectivity.TransitivelyConnected(8, 9)); //true
  EXPECT_FALSE(trajectory_connectivity.TransitivelyConnected(0, 9));//联通0-9

  trajectory_connectivity.Connect(1, 8);                            //联通1-8  
  for (int i : {0, 1}) {
    for (int j : {8, 9}) {
      EXPECT_TRUE(trajectory_connectivity.TransitivelyConnected(i, j));//均联通
    }
  }
}

TEST(TrajectoryConnectivityTest, EmptyConnectedComponents) {
  TrajectoryConnectivity trajectory_connectivity;
  auto connections = trajectory_connectivity.ConnectedComponents();
  EXPECT_EQ(0, connections.size());                    //0分量
}

TEST(TrajectoryConnectivityTest, ConnectedComponents) {
  TrajectoryConnectivity trajectory_connectivity;
  for (int i = 0; i <= 4; ++i) {
    trajectory_connectivity.Connect(0, i); //联通0-1,0-2,0-3,0-4
  }
  for (int i = 5; i <= 9; ++i) {
    trajectory_connectivity.Connect(5, i);//联通5-6,5-7,5-8,5-9
  }
  auto connections = trajectory_connectivity.ConnectedComponents();
  ASSERT_EQ(2, connections.size());                    //2个分量/分组


  //connections的顺序未定 ，所以所以需要使用find函数。          
  // The clustering is arbitrary; we need to figure out which one is which.
  const std::vector<int>* zero_cluster = nullptr;
  const std::vector<int>* five_cluster = nullptr;
  if (std::find(connections[0].begin(), connections[0].end(), 0) !=
      connections[0].end()) {
    zero_cluster = &connections[0];
    five_cluster = &connections[1];
  } else {
    zero_cluster = &connections[1];
    five_cluster = &connections[0];
  }
  for (int i = 0; i <= 9; ++i) {
    //查找联通分量
    EXPECT_EQ(i <= 4, std::find(zero_cluster->begin(), zero_cluster->end(),
                                i) != zero_cluster->end());
    EXPECT_EQ(i > 4, std::find(five_cluster->begin(), five_cluster->end(), i) !=
                         five_cluster->end());
  }
}

TEST(TrajectoryConnectivityTest, ConnectionCount) {
  TrajectoryConnectivity trajectory_connectivity;

  //对同一个连线调用多次"Connect()"函数
  for (int i = 0; i < kNumTrajectories; ++i) {
    trajectory_connectivity.Connect(0, 1);
    // Permute the arguments to check invariance.
    EXPECT_EQ(i + 1, trajectory_connectivity.ConnectionCount(1, 0));
  }
  for (int i = 1; i < 9; ++i) {
    EXPECT_EQ(0, trajectory_connectivity.ConnectionCount(i, i + 1));
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
