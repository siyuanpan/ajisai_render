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

#ifndef AJISAI_INTEGRATORS_BIDIRUTIL_H_
#define AJISAI_INTEGRATORS_BIDIRUTIL_H_

#include <Ajisai/Core/Geometry.h>
#include <Ajisai/Integrators/Integrator.h>
#include <Ajisai/Math/Math.h>
#include <Ajisai/PluginManager/AbstractManager.h>

namespace Ajisai::Integrators {

class BDPTIntegrator : public Integrator {
 public:
  explicit BDPTIntegrator(PluginManager::AbstractManager& manager,
                          const std::string plugin)
      : Integrator{manager, plugin} {}

  struct Vertex {
    Math::Spectrum throughput;
    uint32_t length;

    Core::SurfaceInteraction si;
    Math::Vector3f inDir;

    float DVCM;
    float DVC;
  };

  struct PathState {
    Math::Vector3f origin;
    Math::Vector3f direction;
    Math::Spectrum throughput;
    uint PathLength : 30;
    bool isFiniteLight : 1;
    bool SpecularPath : 1;

    float DVCM;
    float DVC;
  };

  static PathState SampleLightSource(Core::Scene* scene,
                                     Core::Sampler* sampler);

  static Math::Spectrum ConnectToCamera(Core::Scene* scene,
                                        Core::Camera* camera,
                                        Core::Sampler* sampler,
                                        Core::SurfaceInteraction& si,
                                        Vertex& lightVertex,
                                        Math::Vector2f& pRaster);

  static int GenerateLightPath(Core::Scene* scene, Core::Sampler* sampler,
                               const int maxDepth, Vertex* lightVertices,
                               Core::Camera* camera, int* vertexCount,
                               const int rrDepth = rrDepth,
                               const bool connectToCamera = true);

  static void SampleCamera(Core::Camera* camera, Core::Ray& ray,
                           Core::Sampler* sampler, PathState& initPathState);

  static Math::Spectrum HittingLightSource(Core::Scene* scene, Core::Ray& ray,
                                           Core::Intersection& isect,
                                           Core::AreaLight* light,
                                           PathState& cameraPathState);

  static Math::Spectrum ConnectToLight(const Core::Scene* scene,
                                       Core::Ray& pathRay,
                                       const Core::SurfaceInteraction& si,
                                       Core::Sampler* sampler,
                                       PathState& cameraPathState);

  static Math::Spectrum ConnectVertex(Core::Scene* scene,
                                      Core::SurfaceInteraction& si,
                                      const Vertex& lightVertex,
                                      PathState& cameraState);

  virtual Math::Spectrum Li(Core::Scene* scene, Core::Camera* camera,
                            const Math::Vector2i& raster,
                            Core::Sampler* sampler) const override;

  virtual void Render(Core::Scene* scene, Core::Camera* camera,
                      Core::Sampler* sampler) const override;

 private:
  // int rrDepth = 5, maxDepth = 16;
  static constexpr int rrDepth = 5, maxDepth = 16;

  // heuristic
  static constexpr int pow = 1;
};

}  // namespace Ajisai::Integrators

#endif