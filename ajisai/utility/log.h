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
#pragma once
#include <ajisai/ajisai.h>
#include <ajisai/utility/ptr.h>
#include <spdlog/spdlog.h>

AJ_BEGIN

class AJISAI_API Log {
 public:
  static Log& Inst();

  spdlog::logger& GetLogger() { return *core_logger_; }

 private:
  Log();
  Ptr<spdlog::logger> core_logger_;
};

AJ_END

#define AJ_TRACE(...) ::aj::Log::Inst().GetLogger().trace(__VA_ARGS__)
#define AJ_DEBUG(...) ::aj::Log::Inst().GetLogger().debug(__VA_ARGS__)
#define AJ_INFO(...) ::aj::Log::Inst().GetLogger().info(__VA_ARGS__)
#define AJ_WARN(...) ::aj::Log::Inst().GetLogger().warn(__VA_ARGS__)
#define AJ_ERROR(...) ::aj::Log::Inst().GetLogger().error(__VA_ARGS__)
#define AJ_CRITICAL(...) ::aj::Log::Inst().GetLogger().critical(__VA_ARGS__)