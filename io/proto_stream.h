
#ifndef CARTOGRAPHER_IO_PROTO_STREAM_H_
#define CARTOGRAPHER_IO_PROTO_STREAM_H_

#include <fstream>

#include "cartographer/common/port.h"

namespace cartographer {
namespace io {


/*
ProtoStreamWriter类用于将压缩的序列化数据protocol messages 写入文件

ProtoStreamReader类用于读取上述已经compressed的文件
*/
// A simple writer of a compressed sequence of protocol buffer messages to a
// file. The format is not intended to be compatible with any other format used
// outside of Cartographer.
//
// TODO(whess): Compress the file instead of individual messages for better
// compression performance? Should we use LZ4?
class ProtoStreamWriter {
 public:
  ProtoStreamWriter(const string& filename);
  ~ProtoStreamWriter();

  ProtoStreamWriter(const ProtoStreamWriter&) = delete;
  ProtoStreamWriter& operator=(const ProtoStreamWriter&) = delete;

  // Serializes, compressed and writes the 'proto' to the file.
  template <typename MessageType>
  void WriteProto(const MessageType& proto) {
    string uncompressed_data;
    proto.SerializeToString(&uncompressed_data);
    Write(uncompressed_data); //Write()函数执行压缩过程。
  }

  // This should be called to check whether writing was successful.
  bool Close();

 private:
  void Write(const string& uncompressed_data);

  std::ofstream out_;
};

// A reader of the format produced by ProtoStreamWriter.
class ProtoStreamReader {
 public:
  ProtoStreamReader(const string& filename);
  ~ProtoStreamReader();

  ProtoStreamReader(const ProtoStreamReader&) = delete;
  ProtoStreamReader& operator=(const ProtoStreamReader&) = delete;

  template <typename MessageType>
  bool ReadProto(MessageType* proto) {
    string decompressed_data;
    return Read(&decompressed_data) &&
           proto->ParseFromString(decompressed_data);
  }

 private:
  bool Read(string* decompressed_data);

  std::ifstream in_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_PROTO_STREAM_H_
