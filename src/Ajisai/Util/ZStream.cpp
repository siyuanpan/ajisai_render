/*
Copyright 2021 Siyuan Pan <pansiyuan.cs@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "ZStream.h"

#include <zlib.h>
#include <cassert>

namespace Ajisai::Util {

ZStream::ZStream(Stream* child_stream, EStreamType stream_type, int level)
    : child_stream_(child_stream),
      deflate_stream_(new z_stream()),
      inflate_stream_(new z_stream()),
      did_write_(false) {
  deflate_stream_->zalloc = Z_NULL;
  deflate_stream_->zfree = Z_NULL;
  deflate_stream_->opaque = Z_NULL;

  int window_bits = 15 + (stream_type == eGZipStream ? 16 : 0);

  int retval = deflateInit2(deflate_stream_.get(), level, Z_DEFLATED,
                            window_bits, 8, Z_DEFAULT_STRATEGY);

  if (retval != Z_OK)
    throw std::runtime_error("Could not initialize ZLIB: error code ");

  inflate_stream_->zalloc = Z_NULL;
  inflate_stream_->zfree = Z_NULL;
  inflate_stream_->opaque = Z_NULL;
  inflate_stream_->avail_in = 0;
  inflate_stream_->next_in = Z_NULL;

  retval = inflateInit2(inflate_stream_.get(), window_bits);
  if (retval != Z_OK)
    throw std::runtime_error("Could not initialize ZLIB: error code ");
}

ZStream::~ZStream() { Close(); }

void ZStream::Close() {
  if (!child_stream_) return;

  if (did_write_) {
    deflate_stream_->avail_in = 0;
    deflate_stream_->next_in = NULL;
    int output_size = 0;

    do {
      deflate_stream_->avail_out = sizeof(deflate_buffer_);
      deflate_stream_->next_out = deflate_buffer_;

      int retval = deflate(deflate_stream_.get(), Z_FINISH);
      if (retval == Z_STREAM_ERROR)
        throw std::runtime_error("deflate(): stream error!");

      output_size = sizeof(deflate_buffer_) - deflate_stream_->avail_out;

      child_stream_->Write(deflate_buffer_, output_size);
    } while (output_size != 0);
  }

  deflateEnd(deflate_stream_.get());
  inflateEnd(inflate_stream_.get());

  child_stream_ = nullptr;
}

bool ZStream::Opened() { return false; }

void ZStream::Read(void* ptr, size_t size) {
  assert(child_stream_ != nullptr);

  uint8_t* targetPtr = (uint8_t*)ptr;
  while (size > 0) {
    if (inflate_stream_->avail_in == 0) {
      size_t remaining = child_stream_->Size() - child_stream_->Tell();
      inflate_stream_->next_in = inflate_buffer_;
      inflate_stream_->avail_in =
          (uint32_t)std::min(remaining, sizeof(inflate_buffer_));
      if (inflate_stream_->avail_in == 0)
        throw std::runtime_error(
            "Read less data than expected ( more bytes required)");
      child_stream_->Read(inflate_buffer_, inflate_stream_->avail_in);
    }

    inflate_stream_->avail_out = (uint32_t)size;
    inflate_stream_->next_out = targetPtr;

    int retval = inflate(inflate_stream_.get(), Z_NO_FLUSH);
    switch (retval) {
      case Z_STREAM_ERROR:
        throw std::runtime_error("inflate(): stream error!");
        break;
      case Z_NEED_DICT:
        throw std::runtime_error("inflate(): need dictionary!");
        break;
      case Z_DATA_ERROR:
        throw std::runtime_error("inflate(): data error!");
        break;
      case Z_MEM_ERROR:
        throw std::runtime_error("inflate(): memory error!");
        break;
    };

    size_t output_size = size - (size_t)inflate_stream_->avail_out;
    targetPtr += output_size;
    size -= output_size;

    if (size > 0 && retval == Z_STREAM_END)
      throw std::runtime_error(
          "inflate(): attempting to read past the end of the stream!");
  }
}

void ZStream::Write(const void* ptr, size_t size) {
  assert(child_stream_ != nullptr);

  deflate_stream_->avail_in = (uint32_t)size;
  deflate_stream_->next_in = (uint8_t*)ptr;

  int output_size = 0;

  do {
    deflate_stream_->avail_out = sizeof(deflate_buffer_);
    deflate_stream_->next_out = deflate_buffer_;

    int retval = deflate(deflate_stream_.get(), Z_NO_FLUSH);
    if (retval == Z_STREAM_ERROR)
      throw std::runtime_error("deflate(): stream error!");

    output_size = sizeof(deflate_buffer_) - deflate_stream_->avail_out;

    child_stream_->Write(deflate_buffer_, output_size);
  } while (output_size != 0);

  assert(deflate_stream_->avail_in == 0);
  did_write_ = true;
}

void ZStream::Seek(size_t pos) {
  throw std::runtime_error("seek(): unsupported in a ZLIB stream!");
}

size_t ZStream::Tell() const {
  throw std::runtime_error("tell(): unsupported in a ZLIB stream!");
  return 0;
}

size_t ZStream::Size() const {
  throw std::runtime_error("size(): unsupported in a ZLIB stream!");
  return 0;
}

}  // namespace Ajisai::Util