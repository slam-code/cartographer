
#include "cartographer/io/proto_stream.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

class ProtoStreamTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const string tmpdir = P_tmpdir;
    test_directory_ = tmpdir + "/proto_stream_test_XXXXXX";
    ASSERT_NE(mkdtemp(&test_directory_[0]), nullptr) << strerror(errno);
  }

  void TearDown() override { remove(test_directory_.c_str()); }

  string test_directory_;
};

TEST_F(ProtoStreamTest, WriteAndReadBack) {
  const string test_file = test_directory_ + "/test_trajectory.pbstream";
  {
    ProtoStreamWriter writer(test_file);
    for (int i = 0; i != 10; ++i) {
      mapping::proto::Trajectory trajectory;
      trajectory.add_node()->set_timestamp(i);
      writer.WriteProto(trajectory);
    }
    ASSERT_TRUE(writer.Close());
  }
  {
    ProtoStreamReader reader(test_file);
    for (int i = 0; i != 10; ++i) {
      mapping::proto::Trajectory trajectory;
      ASSERT_TRUE(reader.ReadProto(&trajectory));
      ASSERT_EQ(1, trajectory.node_size());
      EXPECT_EQ(i, trajectory.node(0).timestamp());
    }
    mapping::proto::Trajectory trajectory;
    EXPECT_FALSE(reader.ReadProto(&trajectory));
  }
  remove(test_file.c_str());
}

}  // namespace
}  // namespace io
}  // namespace cartographer
