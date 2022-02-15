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
#include "Allocator.h"

#include <fstream>
#include <limits>
#include <cstring>

namespace Ajisai::Util {

namespace {
inline std::ios::openmode IosFlag(FileStream::Mode mode) {
  switch (mode) {
    case FileStream::Mode::eRead:
      return std::ios::binary | std::ios::in;
    case FileStream::Mode::eReadWrite:
      return std::ios::binary | std::ios::in | std::ios::out;
    case FileStream::Mode::eTruncReadWrite:
      return std::ios::binary | std::ios::in | std::ios::out | std::ios::trunc;
    default:
      throw std::runtime_error("Internal error");
  }
}
}  // namespace

FileStream::FileStream(const fs::path& p, Mode mode)
    : mode_(mode), path_(p), file_(new std::fstream) {
  file_->open(p.string(), IosFlag(mode_));

  if (!file_->good()) {
    throw std::runtime_error("I/O error while attempting to open file");
  }
}

FileStream::~FileStream() { Close(); }

void FileStream::Close() { file_->close(); }

bool FileStream::Opened() { return file_ != nullptr; }

void FileStream::Read(void* buffer, size_t size) {
  file_->read((char*)buffer, size);

  if (!file_->good()) {
    bool eof = file_->eof();
    size_t gcount = file_->gcount();
    file_->clear();
    if (eof) {
      throw std::runtime_error("EOF error");
    } else {
      throw std::runtime_error("I/O error while attempting to read");
    }
  }
}

void FileStream::Write(const void* buffer, size_t size) {
  file_->write((char*)buffer, size);

  if (!file_->good()) {
    file_->clear();
    throw std::runtime_error("I/O error while attempting to write");
  }
}

void FileStream::Seek(size_t pos) {
  file_->seekg(static_cast<std::ios::pos_type>(pos));

  if (!file_->good()) {
    throw std::runtime_error("I/O error while attempting to seek to offset");
  }
}

size_t FileStream::Tell() const {
  std::ios::pos_type pos = file_->tellg();
  if (pos == std::ios::pos_type(-1))
    throw std::runtime_error(
        "I/O error while attempting to determine "
        "position in file");
  return static_cast<size_t>(pos);
}

size_t FileStream::Size() const { return fs::file_size(path_); }

// Starts MemoryStream implementation.
const size_t MemoryStream::kBufferSizeIncrement = 16 << 10;
const size_t MemoryStream::kMaxSize = std::numeric_limits<int>::max();

MemoryStream::MemoryStream()
    : buffer_(nullptr), alloc_size_(0), end_(0), tell_(0) {}

MemoryStream::~MemoryStream() {
  Allocator::default_allocator()->Deallocate(buffer_);
  buffer_ = nullptr;
}

bool MemoryStream::Opened() { return true; }

void MemoryStream::Read(void* buffer, size_t size) {
  if (tell_ > end_ || size > kMaxSize) {
    throw std::runtime_error(
        "cannot set file position beyond the end of the file or cannot exceed "
        "the maximum Stream size");
  }

  const int read_size = std::min(end_ - tell_, static_cast<int>(size));
  std::memcpy(buffer, buffer_ + tell_, read_size);
  tell_ += read_size;
}

void MemoryStream::Write(const void* buffer, size_t _size) {
  if (_size > kMaxSize || tell_ > static_cast<int>(kMaxSize - _size)) {
    throw std::runtime_error("A write cannot exceed the maximum Stream size.");
  }

  if (tell_ > end_) {
    if (!Resize(tell_)) {
      throw std::runtime_error("memoryStream resize error");
    }
    const size_t gap = tell_ - end_;
    std::memset(buffer_ + end_, 0, gap);
    end_ = tell_;
  }

  const int size = static_cast<int>(_size);
  const int tell_end = tell_ + size;
  if (Resize(tell_end)) {
    end_ = std::max(tell_end, end_);
    std::memcpy(buffer_ + tell_, buffer, _size);
    tell_ += size;
  } else {
    throw std::runtime_error("write error");
  }
}

void MemoryStream::Seek(size_t pos) { tell_ = static_cast<int>(pos); }

size_t MemoryStream::Tell() const { return tell_; }

size_t MemoryStream::Size() const { return static_cast<size_t>(end_); }

bool MemoryStream::Resize(size_t _size) {
  if (_size > alloc_size_) {
    // Resize to the next multiple of kBufferSizeIncrement, requires
    // kBufferSizeIncrement to be a power of 2.
    static_assert(
        (MemoryStream::kBufferSizeIncrement & (kBufferSizeIncrement - 1)) == 0,
        "kBufferSizeIncrement must be a power of 2");
    const size_t new_size = Align(_size, kBufferSizeIncrement);
    char* new_buffer = reinterpret_cast<char*>(
        Allocator::default_allocator()->Allocate(new_size, 16));
    std::memcpy(new_buffer, buffer_, alloc_size_);
    Allocator::default_allocator()->Deallocate(buffer_);
    buffer_ = new_buffer;
    alloc_size_ = new_size;
  }
  return _size == 0 || buffer_ != nullptr;
}

}  // namespace Ajisai::Util