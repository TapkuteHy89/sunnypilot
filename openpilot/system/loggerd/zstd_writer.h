#pragma once

#include <zstd.h>
#include <cstdint>

#include <string>
#include <vector>
#include <capnp/common.h>

class ZstdFileWriter {
public:
  ZstdFileWriter(const std::string &filename, int compression_level);
  ~ZstdFileWriter();
  void write(void* data, size_t size);
  void flush();
  inline void write(kj::ArrayPtr<capnp::byte> array) { write(array.begin(), array.size()); }

private:
  void flushCache(bool last_chunk);
  void syncFile();

  size_t input_cache_capacity_ = 0;
  std::vector<char> input_cache_;
  std::vector<char> output_buffer_;
  ZSTD_CStream *cstream_;
  int compression_level_ = ZSTD_CLEVEL_DEFAULT;
  FILE* file_ = nullptr;
  uint64_t last_sync_ns_ = 0;
};
