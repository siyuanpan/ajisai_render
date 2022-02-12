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
#include <ajisai/ajisai.h>
#include <ajisai/core/renderer/renderer.h>
#include <ajisai/core/sampler/sampler.h>
#include <ajisai/core/camera/camera.h>
#include <ajisai/core/scene/scene.h>
#include <ajisai/core/medium/medium.h>
#include <ajisai/core/light/area_light.h>
#include <ajisai/core/film.h>
#include <ajisai/core/parallel.h>
#include <ajisai/core/intersection.h>
#include <ajisai/core/renderer/helper.h>
#include <ajisai/math/vector2.h>
#include <ajisai/math/spectrum.h>

AJ_BEGIN

void TiledIntegrator::Render(Scene* scene, Camera* camera, Film* film,
                             Sampler* sampler) {
  auto n_tiles =
      (film->Dimension() + Vector2i{tile_size_ - 1}) / Vector2i{tile_size_};
  auto beginTime = std::chrono::high_resolution_clock::now();
  parallel_for_2d(n_tiles, [=](Vector2i tile_pos, uint32_t tid) {
    (void)tid;
    Bounds2i tile_bounds = Bounds2i{tile_pos * (int)tile_size_,
                                    (tile_pos + Vector2i(1)) * (int)tile_size_};
    auto tile = film->GetTile(tile_bounds);
    auto tile_sampler = sampler->Copy();
    // for (int y = tile.bounds.min().y(); y < tile.bounds.max().y(); ++y) {
    //   for (int x = tile.bounds.min().x(); x < tile.bounds.max().x(); ++x) {
    for (int y = tile_bounds.min().y(); y < tile_bounds.max().y(); ++y) {
      for (int x = tile_bounds.min().x(); x < tile_bounds.max().x(); ++x) {
        tile_sampler->SetSeed(x + y * film->Dimension().x());
        for (int s = 0; s < spp_; ++s) {
          const float u = x + tile_sampler->Next1D();
          const float v = y + tile_sampler->Next1D();
          auto ray =
              camera->GenerateRay(Vector2f{u, v}, tile_sampler->Next2D());
          auto li = Li(ray, scene, tile_sampler.get());
          tile.AddSample(Vector2i{x, y}, li.value, 1.f);
        }
      }
    }
    std::lock_guard<std::mutex> lk(mutex);
    film->MergeTile(tile);
  });
  auto endTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = (endTime - beginTime);
  std::cout << "Rendering done in " << elapsed.count() << "  secs\n";
}

class PathTracing : public TiledIntegrator {
  using LiImpl = std::function<RenderPixel(const Ray&, const Scene*, Sampler*)>;

 public:
  explicit PathTracing(const PTRendererArgs& args)
      : TiledIntegrator(args.spp, args.tile_size),
        min_bounces_{args.min_bounces},
        max_bounces_{args.max_bounces},
        cont_prob_{args.cont_prob},
        use_mis_{args.use_mis},
        specular_depth_{args.specular_depth} {
    func_ = std::bind(use_mis_ ? &PathTracing::TraceMIS : &PathTracing::Trace,
                      this, std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3);
  }

  virtual RenderPixel Li(const Ray& ray, const Scene* scene,
                         Sampler* sampler) const override {
    return func_(ray, scene, sampler);
  }

 private:
  RenderPixel TraceMIS(const Ray& ray, const Scene* scene, Sampler* sampler) {
    RenderPixel pixel{};
    Spectrum throughput{1.f};
    Ray path_ray = ray;

    bool specular_bounce = true;
    int scattering_count = 0;
    for (auto bounce = 0; bounce < max_bounces_; bounce++) {
      if (bounce > min_bounces_) {
        if (sampler->Next1D() > cont_prob_) return pixel;
        throughput /= cont_prob_;
      }

      PrimitiveIntersection inct;
      bool intersected = scene->Intersect(path_ray, &inct);
      if (!intersected) {
        return pixel;
      }

      const auto sp = inct.material->Shade(inct);
      if (bounce == 0) {
        pixel.normal = sp.shading_normal;
        pixel.albedo = sp.bsdf->Albedo();
      }

      const auto medium = inct.WrMedium();
      if (scattering_count < medium->GetMaxScatteringCount()) {
        const auto medium_sample = medium->SampleScattering(
            path_ray.o, inct.pos, sampler, scattering_count > 0);

        throughput *= medium_sample.throughput;

        if (medium_sample.ScatteringHappened()) {
          ++scattering_count;

          const auto& scattering_point = medium_sample.scattering_point;
          const auto phase_function = medium_sample.phase_function;

          //   const auto phase_sample = phase_function->SampleAll(
          //       inct.wr, TransMode::Radiance, sampler->Next3D());
          //   if (!phase_sample.f) return pixel;

          //   coef *= phase_sample.f / phase_sample.pdf;
          //   r = Ray(scattering_point.pos, phase_sample.dir);

          //   continue;
        }
      } else {
        const Spectrum ab = medium->Absorbtion(path_ray.o, inct.pos, sampler);
        throughput *= ab;
      }

      scattering_count = 0;

      if (bounce == 0) {
        if (auto light = inct.primitive->AsLight()) {
          pixel.value +=
              throughput *
              light->Radiance(inct.pos, inct.geometry_normal, inct.uv, inct.wr);
        }
      }

      // direct illumination
      Spectrum direct_illum{};
      for (auto i = 0; i < kDirectIllumSampleCount; ++i) {
        for (auto light : scene->Lights()) {
          // direct_illum +=
          //     throughput *
          //     MISSampleAreaLight(scene, light->AsArea(), inct, sp, sampler);
          direct_illum +=
              throughput * MISSampleLight(scene, light, inct, sp, sampler);
        }
        direct_illum += throughput * MISSampleBsdf(scene, inct, sp, sampler);
      }

      pixel.value += 1.f / kDirectIllumSampleCount * direct_illum;

      const auto bsdf_sample =
          sp.bsdf->SampleAll(inct.wr, TransMode::Radiance, sampler->Next3D());
      if (bsdf_sample.f.IsBlack() || bsdf_sample.pdf < 0.f) return pixel;
      // const auto& normal = inct.shading_normal;
      const auto& normal = inct.geometry_normal;
      throughput *= bsdf_sample.f * std::abs(dot(normal, bsdf_sample.dir)) /
                    bsdf_sample.pdf;
      path_ray = Ray{inct.EpsOffset(bsdf_sample.dir), bsdf_sample.dir};
      specular_bounce = bsdf_sample.is_delta;
    }
    return pixel;
  }

  RenderPixel Trace(const Ray& ray, const Scene* scene, Sampler* sampler) {
    RenderPixel pixel{};
    Spectrum throughput{1.f};
    Ray path_ray = ray;

    bool specular_bounce = true;
    int scattering_count = 0;
    for (auto bounce = 0; bounce < max_bounces_; bounce++) {
      if (bounce > min_bounces_) {
        if (sampler->Next1D() > cont_prob_) return pixel;
        throughput /= cont_prob_;
      }

      PrimitiveIntersection inct;
      bool intersected = scene->Intersect(path_ray, &inct);
      if (!intersected) {
        return pixel;
      }

      const auto sp = inct.material->Shade(inct);
      if (bounce == 0) {
        pixel.normal = sp.shading_normal;
        pixel.albedo = sp.bsdf->Albedo();
      }

      const auto medium = inct.WrMedium();
      if (scattering_count < medium->GetMaxScatteringCount()) {
        const auto medium_sample = medium->SampleScattering(
            path_ray.o, inct.pos, sampler, scattering_count > 0);

        throughput *= medium_sample.throughput;

        if (medium_sample.ScatteringHappened()) {
          ++scattering_count;

          const auto& scattering_point = medium_sample.scattering_point;
          const auto phase_function = medium_sample.phase_function;

          //   const auto phase_sample = phase_function->SampleAll(
          //       inct.wr, TransMode::Radiance, sampler->Next3D());
          //   if (!phase_sample.f) return pixel;

          //   coef *= phase_sample.f / phase_sample.pdf;
          //   r = Ray(scattering_point.pos, phase_sample.dir);

          //   continue;
        }
      } else {
        const Spectrum ab = medium->Absorbtion(path_ray.o, inct.pos, sampler);
        throughput *= ab;
      }

      scattering_count = 0;

      if (auto light = inct.primitive->AsLight()) {
        pixel.value +=
            throughput *
            light->Radiance(inct.pos, inct.geometry_normal, inct.uv, inct.wr);
      }

      // direct illumination
      Spectrum direct_illum;
      for (auto i = 0; i < kDirectIllumSampleCount; ++i) {
        for (auto light : scene->Lights()) {
          //
        }
      }

      const auto bsdf_sample =
          sp.bsdf->SampleAll(inct.wr, TransMode::Radiance, sampler->Next3D());
      if (bsdf_sample.f.IsBlack() || bsdf_sample.pdf < 0.f) return pixel;
      throughput *= bsdf_sample.f *
                    std::abs(dot(inct.geometry_normal, bsdf_sample.dir)) /
                    bsdf_sample.pdf;
      path_ray = Ray{inct.EpsOffset(bsdf_sample.dir), bsdf_sample.dir};
      specular_bounce = bsdf_sample.is_delta;
    }
    return pixel;
  }

 private:
  int min_bounces_;
  int max_bounces_;
  float cont_prob_;
  bool use_mis_;
  int specular_depth_;
  LiImpl func_;
  static const int kDirectIllumSampleCount = 1;
};

Rc<Renderer> CreatePTRenderer(const PTRendererArgs& args) {
  return RcNew<PathTracing>(args);
}

AJ_END