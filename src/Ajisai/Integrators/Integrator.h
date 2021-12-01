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

// #include <future>
// #include <mutex>

// #include "Ajisai/Core/Camera.h"
#include <Ajisai/Core/Film.h>
// #include "Ajisai/Core/Mesh.h"
// #include "Ajisai/Core/Parallel.hpp"
// #include "Ajisai/Core/Sampler.h"
#include <Ajisai/Core/Scene.h>
#include <Ajisai/Math/Math.h>
#include <Ajisai/PluginManager/AbstractPlugin.h>
#include <Ajisai/Util/Directory.h>

namespace Ajisai::Integrators {

// struct RenderContext {
//   std::shared_ptr<Ajisai::Core::Camera> camera;
//   std::shared_ptr<Ajisai::Core::Scene> scene;
//   std::shared_ptr<Ajisai::Core::Sampler> sampler;
// };

// class RenderTask {
//  public:

//   virtual ~RenderTask() {}

//   virtual void Wait() = 0;

//   virtual Math::Spectrum Li(Core::Ray ray, Core::Sampler* sampler) = 0;

//   virtual void Start() = 0;

//   virtual std::shared_ptr<const Core::Film> GetFilm() = 0;

// };

class Integrator : public PluginManager::AbstractPlugin {
 public:
  // int spp = 16;
  // explicit Integrator() {}
  explicit Integrator(PluginManager::AbstractManager& manager,
                      const std::string& plugin)
      : PluginManager::AbstractPlugin{manager, plugin} {}
  // virtual std::shared_ptr<RenderTask> CreateRenderTask(
  //     const RenderContext& ctx) = 0;

  virtual Math::Spectrum Li(Core::Ray ray, Core::Scene* scene,
                            Core::Sampler* sampler) const = 0;

  // static std::string Integrator::pluginInterface() {
  //   return "ajisai.integrators.Integrator/0.0.1";
  // }
  static std::string pluginInterface();
  static std::vector<std::filesystem::path> pluginSearchPaths();
  // static std::vector<std::string> Integrator::pluginSearchPaths() {
  //   std::cout << Util::libraryLocation(&pluginInterface) << std::endl;
  //   return {{}};
  // }
};

// class PathIntegrator : public Integrator {
//  public:
//   virtual std::shared_ptr<RenderTask> CreateRenderTask(
//       const RenderContext& ctx) override;

//  private:
//   int spp = 16;
//   int minDepth = 5, maxDepth = 16;
// };
}  // namespace Ajisai::Integrators
#endif