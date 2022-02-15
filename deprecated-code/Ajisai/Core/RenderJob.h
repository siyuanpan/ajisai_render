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

#ifndef AJISAI_CORE_RENDERJOB_H_
#define AJISAI_CORE_RENDERJOB_H_

#include <future>
#include <memory>

#include <Ajisai/Integrators/Integrator.h>
#include <Ajisai/Util/Ptr.h>

namespace Ajisai::Core {

class Scene;
class Sampler;
class Camera;

// struct SimpleRenderJob {

// };

struct RenderJob {
  RenderJob();

  void Run();

  void Join() { future.wait(); }

  struct Context {
    std::shared_ptr<Scene> scene;
    std::shared_ptr<Sampler> sampler;
    std::shared_ptr<Camera> camera;
    Util::Ptr<Integrators::Integrator> integrator;
  };

  Context ctx;

  int spp = 16;
  int rrDepth;
  int maxDepth;

  std::future<void> future;
  std::mutex mutex;
};

}  // namespace Ajisai::Core

#endif