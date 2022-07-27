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
#include <ajisai/core/renderer/photon_mapping.h>
#include <ajisai/core/light/env_light.h>
#include <ajisai/core/light/area_light.h>
#include <ajisai/core/renderer/helper.h>
#include <ajisai/utility/mem_arena.h>

AJ_BEGIN

sppm::VisiblePoint TracerVP(int forward_max_depth, const Scene* scene,
                            const Ray& r, Sampler* sampler,
                            Spectrum& direct_illum, MemoryArena& arena) {
  Spectrum throughput{1.f};
  //   std::cout << "enter TracerVP\n";

  Ray ray = r;
  for (size_t depth = 0; depth < forward_max_depth; ++depth) {
    PrimitiveIntersection inct;
    bool intersected = scene->Intersect(ray, &inct);
    if (!intersected) {
      if (depth == 0) {
        if (auto env_light = scene->GetEnvLight()) {
          direct_illum += throughput * env_light->Radiance(ray.o, ray.d);
        }
      }

      return {{}, {}, {}, nullptr};
    }

    auto sp = inct.material->Shade(inct, arena);
    if (depth == 0) {
      if (auto light = inct.primitive->AsLight()) {
        direct_illum +=
            throughput *
            light->Radiance(inct.pos, inct.geometry_normal, inct.uv, inct.wr);
      }
    }

    Spectrum sum_di{};
    static int kDirectIllumSampleCount = 1;
    for (auto i = 0; i < kDirectIllumSampleCount; ++i) {
      for (auto light : scene->Lights()) {
        sum_di += throughput * MISSampleLight(scene, light, inct, sp, sampler);
      }

      sum_di += throughput * MISSampleBsdf(scene, inct, sp, sampler);
    }

    direct_illum += 1.f / kDirectIllumSampleCount * sum_di;

    // create vp
    if (sp.bsdf->HasDiffuseComponent() || depth == forward_max_depth - 1) {
      if (!throughput.IsFinite()) {
        return {{}, {}, {}, nullptr};
      }

      sppm::VisiblePoint vp{};
      vp.pos = inct.pos;
      vp.throughput = throughput;
      vp.bsdf = sp.bsdf;
      vp.wr = inct.wr;
      sp.bsdf = nullptr;

      return std::move(vp);
    }

    const auto bsdf_sample =
        sp.bsdf->SampleAll(inct.wr, TransMode::Radiance, sampler->Next3D());
    if (bsdf_sample.f.IsBlack() || bsdf_sample.pdf < 0.f)
      return {{}, {}, {}, nullptr};

    const auto& normal = inct.geometry_normal;
    throughput *= bsdf_sample.f * std::abs(dot(normal, bsdf_sample.dir)) /
                  bsdf_sample.pdf;
    ray = Ray{inct.EpsOffset(bsdf_sample.dir), bsdf_sample.dir};
  }

  //   std::cout << "leave TracerVP\n";

  return {{}, {}, {}, nullptr};
}

void TracerPhoton(int min_depth, int max_depth, float cont_prob,
                  sppm::VisiblePointSearcher& vp_searcher, const Scene* scene,
                  Sampler* sampler, MemoryArena& arena) {
  auto [light, light_pdf] = scene->SampleLight(sampler->Next1D());
  if (!light) return;

  const auto emit = light->SampleEmit(sampler);
  if (emit.radiance.IsBlack() || emit.pdf_dir == 0.f || emit.pdf_pos == 0.f)
    return;

  Spectrum coef = emit.radiance * std::abs(dot(emit.normal, emit.dir)) /
                  (light_pdf * emit.pdf_pos * emit.pdf_dir);
  Ray ray(emit.pos, emit.dir);

  for (int depth = 1; depth <= max_depth; ++depth) {
    if (depth > min_depth) {
      if (sampler->Next1D() > cont_prob) return;
      coef /= cont_prob;
    }

    PrimitiveIntersection inct;
    bool intersected = scene->Intersect(ray, &inct);
    if (!intersected) return;

    if (depth > 1) vp_searcher.AddPhoton(inct.pos, coef, inct.wr);

    auto sp = inct.material->Shade(inct, arena);
    const auto bsdf_sample =
        sp.bsdf->SampleAll(inct.wr, TransMode::Radiance, sampler->Next3D());
    if (bsdf_sample.f.IsBlack() || bsdf_sample.pdf < 0.f) return;

    coef *= bsdf_sample.f / bsdf_sample.pdf *
            std::abs(dot(inct.geometry_normal, bsdf_sample.dir));
    ray = Ray(inct.EpsOffset(bsdf_sample.dir), bsdf_sample.dir);
  }
}

void UpdatePixel(float alpha, sppm::Pixel& pixel) {
  assert(pixel.vp.IsValid());

  if (pixel.M > 0) {
    float new_N = pixel.N + alpha * pixel.M;
    float new_R = pixel.radius * std::sqrt(new_N / (pixel.N + pixel.M));

    Spectrum phi{};
    for (size_t i = 0; i < 3; ++i) phi[i] = pixel.phi[i];

    pixel.tau = (pixel.tau + pixel.vp.throughput * phi) * (new_R * new_R) /
                (pixel.radius * pixel.radius);

    pixel.N = new_N;
    pixel.radius = new_R;
    pixel.M = 0;

    for (size_t i = 0; i < 3; ++i) pixel.phi[i] = 0.f;
  }

  pixel.vp.throughput = Spectrum{};
  pixel.vp.bsdf = nullptr;
}

Spectrum ComputePixelRadiance(int iter_count, uint64_t photon_N,
                              const sppm::Pixel& pixel) {
  const Spectrum direct_illm = pixel.direct_illum / float(iter_count);

  const float dem =
      photon_N * Constants<float>::pi() * pixel.radius * pixel.radius;

  const Spectrum photon_illum = pixel.tau / dem;
  return direct_illm + photon_illum;
}

class SPPMRenderer : public Renderer {
 public:
  explicit SPPMRenderer(const SPPMRendererArgs& args) : args_{args} {}

  virtual void Render(Scene* scene, Camera* camera, Film* film,
                      Sampler* sampler) override {
    auto world_bound = scene->WorldBound();

    auto thread_count = std::thread::hardware_concurrency();
    std::vector<MemoryArena> arenas(thread_count);

    float init_radius = args_.init_radius;
    if (init_radius < 0) {
      init_radius = (world_bound.max() - world_bound.min()).length() / 1000.f;
    }

    world_bound.min() -= Vector3f{init_radius};
    world_bound.max() += Vector3f{init_radius};

    Image<sppm::Pixel> sppm_pixel(film->Dimension());
    for (size_t x = 0; x < film->Dimension().x(); ++x)
      for (size_t y = 0; y < film->Dimension().y(); ++y)
        sppm_pixel(x, y).radius = init_radius;

    float max_radius = init_radius;

    for (size_t iter = 0; iter < args_.iteration_count; ++iter) {
      std::cout << "iter : " << iter << std::endl;
      const float grid_sidelen = 1.05f * max_radius;
      sppm::VisiblePointSearcher vp_searcher(world_bound, grid_sidelen, 40960);

      for (auto& arena : arenas) arena.Release();

      auto n_tiles = (film->Dimension() + Vector2i{64 - 1}) / Vector2i{64};
#if 1
      parallel_for_2d(n_tiles, [&](Vector2i tile_pos, uint32_t tid) {
        Bounds2i tile_bounds =
            Bounds2i{tile_pos * (int)64, (tile_pos + Vector2i(1)) * (int)64};
        auto tile_sampler = sampler->Copy();

        for (int y = tile_bounds.min().y(); y < tile_bounds.max().y(); ++y) {
          for (int x = tile_bounds.min().x(); x < tile_bounds.max().x(); ++x) {
            tile_sampler->SetSeed(x + y * film->Dimension().x());
            const float u = x + tile_sampler->Next1D();
            const float v = y + tile_sampler->Next1D();
            auto ray =
                camera->GenerateRay(Vector2f{u, v}, tile_sampler->Next2D());

            sppm::Pixel& pixel = sppm_pixel(x, y);
            pixel.vp =
                TracerVP(args_.forward_max_depth, scene, ray,
                         tile_sampler.get(), pixel.direct_illum, arenas[tid]);
            if (pixel.vp.IsValid()) vp_searcher.AddVp(pixel, arenas[tid]);
          }
        }
      });
#else
      for (int y = 0; y < film->Dimension().y(); ++y) {
        for (int x = 0; x < film->Dimension().x(); ++x) {
          const float u = x + sampler->Next1D();
          const float v = y + sampler->Next1D();
          auto ray = camera->GenerateRay(Vector2f{u, v}, sampler->Next2D());

          sppm::Pixel& pixel = sppm_pixel(x, y);
          std::cout << "before traceVP\n";
          pixel.vp = TracerVP(args_.forward_max_depth, scene, ray, sampler,
                              pixel.direct_illum, arenas[0]);
          std::cout << "after traceVP\n";
          if (pixel.vp.IsValid()) vp_searcher.AddVp(pixel, arenas[0]);
        }
      }
#endif
      //   std::cout << "after forward trace\n";

      // trace photons
      parallel_for(args_.photons_per_iteration, [&](uint32_t i, uint32_t tid) {
        auto photon_sampler = sampler->Copy();
        photon_sampler->SetSeed(i);
        TracerPhoton(args_.photon_min_depth, args_.photon_max_depth,
                     args_.photon_cont_prob, vp_searcher, scene, sampler,
                     arenas[tid]);
      });

      max_radius = 0.f;
      for (size_t x = 0; x < film->Dimension().x(); ++x)
        for (size_t y = 0; y < film->Dimension().y(); ++y) {
          if (sppm_pixel(x, y).vp.IsValid()) {
            max_radius = std::max(max_radius, sppm_pixel(x, y).radius);
          }
        }

      if (max_radius == 0.f) max_radius = init_radius;

      parallel_for(film->Dimension().y(), [&](uint32_t y, uint32_t tid) {
        for (int x = 0; x < film->Dimension().x(); ++x) {
          if (sppm_pixel(x, y).vp.IsValid()) {
            UpdatePixel(args_.update_alpha, sppm_pixel(x, y));
          }
        }
      });
    }

    const uint64_t photon_count =
        uint64_t(args_.iteration_count) * uint64_t(args_.photons_per_iteration);
    Bounds2i tile_bounds = Bounds2i{Vector2i{0, 0}, film->Dimension()};
    Tile tile = film->GetTile(tile_bounds);
    for (int y = tile_bounds.min().y(); y < tile_bounds.max().y(); ++y)
      for (int x = tile_bounds.min().x(); x < tile_bounds.max().x(); ++x) {
        tile.AddSample(Vector2i{x, y},
                       ComputePixelRadiance(args_.iteration_count, photon_count,
                                            sppm_pixel(x, y)),
                       1.f);
      }
    film->MergeTile(tile);
  }

 private:
  SPPMRendererArgs args_;
};

Rc<Renderer> CreateSPPMRenderer(const SPPMRendererArgs& args) {
  return RcNew<SPPMRenderer>(args);
}

AJ_END