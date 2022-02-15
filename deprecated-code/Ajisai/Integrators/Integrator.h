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

#ifndef AJISAI_INTEGRATORS_INTEGRATOR_H_
#define AJISAI_INTEGRATORS_INTEGRATOR_H_

#include <Ajisai/Core/Film.h>

#include <Ajisai/Core/Scene.h>
#include <Ajisai/Math/Math.h>
#include <Ajisai/PluginManager/AbstractPlugin.h>
#include <Ajisai/Util/Directory.h>

namespace Ajisai::Integrators {

class AJISAI_API Integrator : public PluginManager::AbstractPlugin {
 public:
  explicit Integrator(PluginManager::AbstractManager& manager,
                      const std::string& plugin)
      : PluginManager::AbstractPlugin{manager, plugin} {}

  virtual ~Integrator() = default;

  virtual Math::Spectrum Li(Core::Scene* scene, Core::Camera* camera,
                            const Math::Vector2i& raster,
                            Core::Sampler* sampler) const = 0;

  virtual void Render(Core::Scene* scene, Core::Camera* camera,
                      Core::Sampler* sampler) const = 0;

  static std::string pluginInterface();
  static std::vector<std::filesystem::path> pluginSearchPaths();
};

}  // namespace Ajisai::Integrators
#endif