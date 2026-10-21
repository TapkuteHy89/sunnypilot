
#include "system/loggerd/zstd_writer.h"

#include <cassert>
#include <cerrno>
#include <unistd.h>

#include "common/util.h"
#include "common/timing.h"

// Constructor: Initializes compression stream and opens file
ZstdFileWriter::ZstdFileWriter(const std::string& filename, int compression_level) {
  compression_level_ = compression_level;
  // Create the compression stream
  cstream_ = ZSTD_createCStream();
  assert(cstream_);

  size_t initResult = ZSTD_initCStream(cstream_, compression_level);
  assert(!ZSTD_isError(initResult));

  input_cache_capacity_ = ZSTD_CStreamInSize();
  input_cache_.reserve(input_cache_capacity_);
  output_buffer_.resize(ZSTD_CStreamOutSize());

  file_ = util::safe_fopen(filename.c_str(), "wb");
  assert(file_ != nullptr);
}

// Destructor: Finalizes compression and closes file
ZstdFileWriter::~ZstdFileWriter() {
  flushCache(true);
  syncFile();

  int err = fclose(file_);
  assert(err == 0);

  ZSTD_freeCStream(cstream_);
}

// Compresses and writes data to file
void ZstdFileWriter::write(void* data, size_t size) {
  // Add data to the input cache
  input_cache_.insert(input_cache_.end(), (uint8_t*)data, (uint8_t*)data + size);

  // If the cache is full, compress and write to the file
  if (input_cache_.size() >= input_cache_capacity_ || nanos_since_boot() - last_sync_ns_ >= 1000000000ULL) {
    flushCache(false);
  }
}

void ZstdFileWriter::flush() {
  flushCache(false);
  syncFile();
}

void ZstdFileWriter::syncFile() {
  int ret = util::safe_fflush(file_);
  assert(ret == 0);
  do {
    ret = fdatasync(fileno(file_));
  } while (ret == -1 && errno == EINTR);
  assert(ret == 0);
  last_sync_ns_ = nanos_since_boot();
}

// Compress and flush the input cache to the file
void ZstdFileWriter::flushCache(bool last_chunk) {
  if (input_cache_.empty()) {
    return;
  }

  ZSTD_inBuffer input = {input_cache_.data(), input_cache_.size(), 0};
  int finished = 0;

  do {
    ZSTD_outBuffer output = {output_buffer_.data(), output_buffer_.size(), 0};
    // Finish every cache-sized chunk as an independent zstd frame. Concatenated
    // frames are standard zstd streams, and this keeps the file valid at the
    // most recent flush boundary even if Android kills loggerd abruptly.
    size_t remaining = ZSTD_compressStream2(cstream_, &output, &input, ZSTD_e_end);
    assert(!ZSTD_isError(remaining));

    size_t written = util::safe_fwrite(output_buffer_.data(), 1, output.pos, file_);
    assert(written == output.pos);

    finished = remaining == 0;
  } while (!finished);

  input_cache_.clear();
  util::safe_fflush(file_);
  // fflush alone leaves data in the kernel cache: a device restart can leave
  // a persisted file size followed by zero pages. Bound that exposure without
  // synchronizing every cache-sized frame on the logger receive thread.
  if (nanos_since_boot() - last_sync_ns_ >= 1000000000ULL) syncFile();

  if (!last_chunk) {
    size_t init_result = ZSTD_initCStream(cstream_, compression_level_);
    assert(!ZSTD_isError(init_result));
  }
}
