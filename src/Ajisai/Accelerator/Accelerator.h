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

#ifndef AJISAI_ACCELERATOR_H_
#define AJISAI_ACCELERATOR_H_

#include <Ajisai/PluginManager/AbstractPlugin.h>

namespace Ajisai::Core {
class Scene;
}

namespace Ajisai::Accelerator {

class Accel : public PluginManager::AbstractPlugin {
 public:
  Accel(PluginManager::AbstractManager& manager, const std::string& plugin)
      : PluginManager::AbstractPlugin{manager, plugin} {}

  virtual void Build(const Core::Scene* scene) = 0;

  AJISAI_PLUGIN_STATIC_FUNC()
};

}  // namespace Ajisai::Accelerator

#endif