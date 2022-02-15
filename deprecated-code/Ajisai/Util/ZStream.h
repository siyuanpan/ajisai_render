#pragma once

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

#include "Stream.h"

extern "C" {
struct z_stream_s;
typedef struct z_stream_s z_stream;
};

namespace Ajisai::Util {

namespace {
constexpr size_t kZStreamBufferSize = 32768;
}

class ZStream : public Stream {
 public:
  enum EStreamType {
    eDeflateStream,  ///< A raw deflate stream
    eGZipStream      ///< A gzip-compatible stream
  };

  ZStream(Stream* child_stream, EStreamType stream_type = eDeflateStream,
          int level = -1);

  virtual ~ZStream();

  void Close();

  virtual bool Opened() override;

  virtual void Read(void* buffer, size_t size) override;

  virtual void Write(const void* buffer, size_t size) override;

  virtual void Seek(size_t offset) override;

  virtual size_t Tell() const override;

  virtual size_t Size() const override;

 private:
  Stream* child_stream_;
  std::unique_ptr<z_stream> deflate_stream_, inflate_stream_;
  uint8_t deflate_buffer_[kZStreamBufferSize];
  uint8_t inflate_buffer_[kZStreamBufferSize];
  bool did_write_;
};
}  // namespace Ajisai::Util