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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <filesystem>

namespace fs = std::filesystem;

namespace Ajisai::Util {

class Stream {
 public:
  virtual bool Opened() = 0;

  virtual void Read(void* buffer, size_t size) = 0;

  virtual void Write(const void* buffer, size_t size) = 0;

  enum Origin { eBegin, eEnd, eCurrent };

  virtual void Seek(size_t offset) = 0;

  virtual size_t Tell() const = 0;

  virtual size_t Size() const = 0;

  Stream(const Stream&) = delete;
  Stream& operator=(const Stream&) = delete;

 protected:
  Stream() {}

  virtual ~Stream() {}
};

class FileStream : public Stream {
 public:
  enum Mode { eRead, eReadWrite, eTruncReadWrite };

  FileStream(const fs::path& p, Mode mode = eRead);

  virtual ~FileStream();

  void Close();

  virtual bool Opened() override;

  virtual void Read(void* buffer, size_t size) override;

  virtual void Write(const void* buffer, size_t size) override;

  virtual void Seek(size_t offset) override;

  virtual size_t Tell() const override;

  virtual size_t Size() const override;

 private:
  Mode mode_;
  fs::path path_;
  mutable std::unique_ptr<std::fstream> file_;
};

class MemoryStream : public Stream {
 public:
  MemoryStream();

  virtual ~MemoryStream();

  virtual bool Opened() override;

  virtual void Read(void* buffer, size_t size) override;

  virtual void Write(const void* buffer, size_t size) override;

  virtual void Seek(size_t offset) override;

  virtual size_t Tell() const override;

  virtual size_t Size() const override;

 private:
  bool Resize(size_t size);

  static const size_t kBufferSizeIncrement;

  static const size_t kMaxSize;

  char* buffer_;

  size_t alloc_size_;

  int end_;

  int tell_;
};

}  // namespace Ajisai::Util