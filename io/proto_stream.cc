
#include "cartographer/io/proto_stream.h"

namespace cartographer {
namespace io {

namespace {

// First eight bytes to identify our proto stream format.
const size_t kMagic = 0x7b1d1f7b5bf501db;//文件头标识符

void WriteSizeAsLittleEndian(size_t size, std::ostream* out) {
  for (int i = 0; i != 8; ++i) {
    out->put(size & 0xff);
    size >>= 8;
  }
}

bool ReadSizeAsLittleEndian(std::istream* in, size_t* size) {
  *size = 0;
  for (int i = 0; i != 8; ++i) {
    *size >>= 8;
    *size += static_cast<size_t>(in->get()) << 56;
  }
  return !in->fail();
}

}  // namespace

ProtoStreamWriter::ProtoStreamWriter(const string& filename)
    : out_(filename, std::ios::out | std::ios::binary) {
  WriteSizeAsLittleEndian(kMagic, &out_);
}

ProtoStreamWriter::~ProtoStreamWriter() {}

void ProtoStreamWriter::Write(const string& uncompressed_data) {
  string compressed_data;
  common::FastGzipString(uncompressed_data, &compressed_data);
  WriteSizeAsLittleEndian(compressed_data.size(), &out_);
  out_.write(compressed_data.data(), compressed_data.size());
}

bool ProtoStreamWriter::Close() {
  out_.close();
  return !out_.fail();
}

ProtoStreamReader::ProtoStreamReader(const string& filename)
    : in_(filename, std::ios::in | std::ios::binary) {
  size_t magic;
  if (!ReadSizeAsLittleEndian(&in_, &magic) || magic != kMagic) {
    in_.setstate(std::ios::failbit);
  }
}

ProtoStreamReader::~ProtoStreamReader() {}

bool ProtoStreamReader::Read(string* decompressed_data) {
  size_t compressed_size;
  if (!ReadSizeAsLittleEndian(&in_, &compressed_size)) {
    return false;
  }
  string compressed_data(compressed_size, '\0');
  if (!in_.read(&compressed_data.front(), compressed_size)) {
    return false;
  }
  common::FastGunzipString(compressed_data, decompressed_data);
  return true;
}

}  // namespace io
}  // namespace cartographer
