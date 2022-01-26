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
#include <mutex>

AJ_BEGIN

class Scene;
class Camera;
class Film;
class Sampler;
struct Pixel;
class Ray;

struct PTRendererArgs {
  int spp;
  int tile_size;
  int min_bounces;
  int max_bounces;
  float cont_prob;
  bool use_mis;
  int specular_depth;
};

class Renderer {
 public:
  virtual ~Renderer() = default;

  virtual void Render(Scene* scene, Camera* camera, Film* film,
                      Sampler* sampler) = 0;
};

class Integrator : public Renderer {
 public:
  virtual Pixel Li(const Ray& ray, const Scene* scene,
                   Sampler* sampler) const = 0;
};

class TiledIntegrator : public Integrator {
 public:
  TiledIntegrator(int spp, int tile_size) : spp_(spp), tile_size_(tile_size) {}

  virtual void Render(Scene* scene, Camera* camera, Film* film,
                      Sampler* sampler) override;

 private:
  int spp_;
  int tile_size_;
  std::mutex mutex;
};

AJISAI_API Rc<Renderer> CreatePTRenderer(const PTRendererArgs&);

AJ_END