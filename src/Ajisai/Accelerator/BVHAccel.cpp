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

#include <Ajisai/Accelerator/Accelerator.h>

namespace Ajisai::Accelerator {

enum class SplitMethod { SAH, HLBVH, Middle, EqualCounts };

class BVHAccel final : public Accel {
 public:
  BVHAccel(PluginManager::AbstractManager& manager, const std::string& plugin)
      : Accel{manager, plugin} {}

  virtual void Build(const Core::Scene* scene) override {}
};

}  // namespace Ajisai::Accelerator

AJISAI_PLUGIN_REGISTER(BVHAccel, Ajisai::Accelerator::BVHAccel,
                       "ajisai.accelerator.Accelerator/0.0.1")