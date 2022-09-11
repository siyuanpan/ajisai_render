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
#include <ajisai/core/renderer/hashgrid.h>
#include <ajisai/core/light/env_light.h>
#include <ajisai/core/light/area_light.h>

AJ_BEGIN

namespace {

struct SubPathState {
  Vector3f origin;
  Vector3f direction;
  Spectrum throughput;
  uint32_t path_length : 30;
  uint32_t is_finite_light : 1;
  uint32_t specular_path : 1;

  float dVCM;
  float dVC;
  float dVM;
};

struct PathVertex {
  Vector3f hit_point;
  Vector3f in_dir;
  Spectrum throughput;
  uint32_t path_length;
  Vector3f normal;

  const BSDF* bsdf;

  float dVCM;
  float dVC;
  float dVM;

  Vector3f GetPosition() const { return hit_point; }
};

struct RangeQuery {
  const PrimitiveIntersection& inct;
  const SubPathState camera_state;
  const BSDF* bsdf;
  float cont_prob;
  int max_path_length;
  int min_path_length;
  float mis_vm_weight_factor;
  float mis_vc_weight_factor;
  Spectrum contrib{};

  RangeQuery(const PrimitiveIntersection& inct, const SubPathState camera_state,
             const BSDF* bsdf, float cont_prob, int max_path_length,
             int min_path_length, float mis_vm_weight_factor,
             float mis_vc_weight_factor)
      : inct(inct),
        camera_state(camera_state),
        bsdf(bsdf),
        cont_prob(cont_prob),
        max_path_length(max_path_length),
        min_path_length(min_path_length),
        mis_vm_weight_factor(mis_vm_weight_factor),
        mis_vc_weight_factor(mis_vc_weight_factor),
        contrib{Spectrum{0.f}} {}

  const Vector3f GetPosition() const { return inct.pos; }

  const Spectrum GetContrib() const { return contrib; }

  float Mis(float value) { return value; }

  void Process(const PathVertex& light_vertex) {
    if ((light_vertex.path_length + camera_state.path_length >
         max_path_length) ||
        (light_vertex.path_length + camera_state.path_length <
         min_path_length)) {
      return;
    }

    const Vector3f light_dir = light_vertex.in_dir;

    const auto bsdf_eval =
        bsdf->EvalAll(light_dir, inct.wr, TransMode::Radiance);

    if (bsdf_eval.IsBlack()) return;

    float camera_dir_pdf_w = bsdf->PdfAll(light_dir, inct.wr);
    float camera_rev_pdf_w = bsdf->PdfAll(inct.wr, light_dir);

    if (camera_dir_pdf_w == 0.f || camera_rev_pdf_w == 0.f) return;

    camera_dir_pdf_w *= cont_prob;
    camera_rev_pdf_w *= cont_prob;

    const float w_light = light_vertex.dVCM * mis_vc_weight_factor +
                          light_vertex.dVM * Mis(camera_dir_pdf_w);

    const float w_camera = camera_state.dVCM * mis_vc_weight_factor +
                           camera_state.dVM * Mis(camera_rev_pdf_w);

    const float mis_weight = 1.f / (w_light + 1.f + w_camera);

    // std::cout << "light_vertex.dVCM : " << light_vertex.dVCM << std::endl;
    // std::cout << "mis_vc_weight_factor : " << mis_vc_weight_factor <<
    // std::endl; std::cout << "light_vertex.dVM : " << light_vertex.dVM <<
    // std::endl; std::cout << "camera_dir_pdf_w : " << camera_dir_pdf_w <<
    // std::endl;

    // std::cout << "w_light : " << w_light << std::endl;

    // std::cout << "camera_state.dVCM : " << camera_state.dVCM << std::endl;
    // std::cout << "camera_state.dVM : " << camera_state.dVM << std::endl;
    // std::cout << "camera_rev_pdf_w : " << camera_rev_pdf_w << std::endl;

    // std::cout << "w_camera : " << w_camera << std::endl;

    // std::cout << "mis_weight : " << mis_weight << std::endl;

    contrib += mis_weight * bsdf_eval * light_vertex.throughput;
  }
};

}  // namespace

class VCMRenderer : public Renderer {
 public:
  explicit VCMRenderer(VCMRendererArgs args) : args_{args} {}

  virtual void Render(Scene* scene, Camera* camera, Film* film,
                      Sampler* sampler) override {
    MemoryArena arena;

    for (size_t i = 0; i < args_.iteration; ++i) {
      arena.Release();

      RenderIter(scene, camera, film, sampler, i, arena);
    }
    // const int path_count = film->Dimension().x() * film->Dimension().y();

    // // Generate light paths
    // for (size_t path_idx = 0; path_idx < path_count; ++path_idx) {
    //   SubPathState light_state{};
    // }
  }

 private:
  void RenderIter(Scene* scene, Camera* camera, Film* film, Sampler* sampler,
                  int iter, MemoryArena& arena) {
    const int path_count = film->Dimension().x() * film->Dimension().y();
    light_sub_path_count_ =
        float(film->Dimension().x() * film->Dimension().y());

    std::cout << "scene->WorldBound().size() : "
              << scene->WorldBound().size().x() << " "
              << scene->WorldBound().size().y() << " "
              << scene->WorldBound().size().z() << std::endl;
    float radius = args_.base_radius * scene->WorldBound().size().min();
    radius /= std::pow(float(iter + 1), 0.5f * (1.f - args_.radius_alpha));
    radius = std::max(radius, 1e-7f);
    const float radius_sqr = radius * radius;

    vm_normalization_ =
        1.f / (radius_sqr * Constants<float>::pi() * light_sub_path_count_);

    std::cout << "light_sub_path_count_ : " << light_sub_path_count_
              << std::endl;
    const float eta_vcm =
        (Constants<float>::pi() * radius_sqr) * light_sub_path_count_;
    mis_vm_weight_factor_ = args_.use_vm ? Mis(eta_vcm) : 0.f;
    mis_vc_weight_factor_ = args_.use_vc ? Mis(1.f / eta_vcm) : 0.f;

    std::cout << "eta_vcm : " << eta_vcm << std::endl;
    std::cout << "mis_vm_weight_factor_ : " << mis_vm_weight_factor_
              << std::endl;
    std::cout << "mis_vc_weight_factor_ : " << mis_vc_weight_factor_
              << std::endl;

    path_ends_.resize(path_count);
    memset(path_ends_.data(), 0, path_ends_.size() * sizeof(int));

    light_vertices_.reserve(path_count);
    light_vertices_.clear();

    // Generate light paths
    for (size_t path_idx = 0; path_idx < path_count; ++path_idx) {
      SubPathState light_state{};
      GenerateLightSample(light_state, scene, sampler);

      for (;; ++light_state.path_length) {
        Ray ray{light_state.origin, light_state.direction};

        PrimitiveIntersection inct;
        if (!scene->Intersect(ray, &inct)) break;

        const auto sp = inct.material->Shade(inct, arena);

        const auto bsdf_sample =
            sp.bsdf->SampleAll(inct.wr, TransMode::Radiance, sampler->Next3D());

        // if (bsdf_sample.f.IsBlack() || bsdf_sample.pdf < 0.f) break;

        {
          if (light_state.path_length > 1 || light_state.is_finite_light == 1) {
            light_state.dVCM *= Mis(inct.t * inct.t);
          }

          float cos_in = std::abs(dot(inct.shading_normal, -ray.d));
          light_state.dVCM /= Mis(std::abs(cos_in));
          light_state.dVC /= Mis(std::abs(cos_in));
          light_state.dVM /= Mis(std::abs(cos_in));
        }

        if (!bsdf_sample.is_delta && (args_.use_vc || args_.use_vm)) {
          PathVertex light_vertex{};
          light_vertex.hit_point = inct.pos;
          light_vertex.in_dir = inct.wr;
          light_vertex.throughput = light_state.throughput;
          light_vertex.path_length = light_state.path_length;
          light_vertex.bsdf = sp.bsdf;
          light_vertex.normal = inct.shading_normal;

          light_vertex.dVCM = light_state.dVCM;
          light_vertex.dVC = light_state.dVC;
          light_vertex.dVM = light_state.dVM;

          light_vertices_.push_back(light_vertex);
        }

        if (!bsdf_sample.is_delta && args_.use_vc) {
          if (light_state.path_length + 1 >= args_.min_path_length) {
            ConnectToCamera(light_state, scene, camera, film, sampler, sp.bsdf,
                            inct);
          }
        }

        if (light_state.path_length + 2 > args_.max_path_length) {
          break;
        }

        if (!SampleScattering(light_state, sampler, sp.bsdf, inct)) break;
      }

      path_ends_[path_idx] = (int)light_vertices_.size();
    }

    // Build hash grid
    if (args_.use_vm) {
      hash_grid_.Reserve(path_count);
      hash_grid_.Build(light_vertices_, radius);
    }

    Bounds2i tile_bounds = Bounds2i{Vector2i{0, 0}, film->Dimension()};
    Tile tile = film->GetTile(tile_bounds);

    // Generate camera paths
    for (size_t path_idx = 0; path_idx < path_count; ++path_idx) {
      SubPathState camera_state{};
      const Vector2f screen_sample =
          GenerateCameraSample(path_idx, camera_state, camera, film, sampler);
      Spectrum color{};
      //   std::cout << "camera_state.dVCM : " << camera_state.dVCM <<
      //   std::endl;

      for (;; ++camera_state.path_length) {
        Ray ray{camera_state.origin, camera_state.direction};

        PrimitiveIntersection inct;
        if (!scene->Intersect(ray, &inct)) {
          if (auto env_light = scene->GetEnvLight()) {
            if (camera_state.path_length >= args_.min_path_length) {
              color +=
                  camera_state.throughput * env_light->Radiance(ray.o, ray.d);
            }
          }
          break;
        }

        const auto sp = inct.material->Shade(inct, arena);

        const auto bsdf_sample =
            sp.bsdf->SampleAll(inct.wr, TransMode::Radiance, sampler->Next3D());

        // if (bsdf_sample.f.IsBlack() || bsdf_sample.pdf < 0.f) break;

        {
          float cos_in = std::abs(dot(inct.shading_normal, -ray.d));
          camera_state.dVCM *= Mis(inct.t * inct.t);
          camera_state.dVCM /= Mis(std::abs(cos_in));
          camera_state.dVC /= Mis(std::abs(cos_in));
          camera_state.dVM /= Mis(std::abs(cos_in));
        }

        if (auto light = inct.primitive->AsLight()) {
          if (camera_state.path_length >= args_.min_path_length) {
            Spectrum radiance = light->Radiance(inct.pos, inct.geometry_normal,
                                                inct.uv, inct.wr);
            if (!radiance.IsBlack()) {
              if (camera_state.path_length == 1)
                color += camera_state.throughput * radiance;
              else if (args_.use_vm && !args_.use_vc) {
                if (camera_state.specular_path)
                  color += camera_state.throughput * radiance;
              } else {
                float direct_pdf_a;
                float emission_pdf_w;
                light->PdfBdpt(inct.wr, inct.pos, inct.geometry_normal,
                               &direct_pdf_a, &emission_pdf_w);

                direct_pdf_a *= args_.cont_prob;
                emission_pdf_w *= args_.cont_prob;

                const float w_camera = Mis(direct_pdf_a) * camera_state.dVCM +
                                       Mis(emission_pdf_w) * camera_state.dVC;

                const float mis_weight = 1.f / (1.f + w_camera);

                color += camera_state.throughput * mis_weight * radiance;
              }
            }

            // color += camera_state.throughput *
            //          light->Radiance(inct.pos, inct.geometry_normal, inct.uv,
            //                          inct.wr);
          }
          break;
        }

        if (camera_state.path_length >= args_.max_path_length) break;

        if (!bsdf_sample.is_delta && args_.use_vc) {
          if (camera_state.path_length + 1 >= args_.min_path_length) {
            color +=
                camera_state.throughput *
                DirectIllumination(camera_state, inct, sp.bsdf, scene, sampler);
          }
        }

        if (!bsdf_sample.is_delta && args_.use_vc) {
          const Vector2i range{(path_idx == 0) ? 0 : path_ends_[path_idx - 1],
                               path_ends_[path_idx]};

          for (size_t i = range.x(); i < range.y(); ++i) {
            const PathVertex& light_vertex = light_vertices_[i];

            if (light_vertex.path_length + 1 + camera_state.path_length <
                args_.min_path_length)
              continue;

            if (light_vertex.path_length + 1 + camera_state.path_length >
                args_.max_path_length)
              continue;

            color += camera_state.throughput * light_vertex.throughput *
                     ConnectVertices(camera_state, light_vertex, scene, inct,
                                     sp.bsdf);
          }
        }

        if (!bsdf_sample.is_delta && args_.use_vm) {
          RangeQuery query{inct,
                           camera_state,
                           sp.bsdf,
                           args_.cont_prob,
                           args_.max_path_length,
                           args_.min_path_length,
                           mis_vm_weight_factor_,
                           mis_vc_weight_factor_};
          //   std::cout << "camera_state.dVCM : " << camera_state.dVCM <<
          //   std::endl;
          hash_grid_.Process(light_vertices_, query);
          //   std::cout << "camera_state.throughput : "
          //             << camera_state.throughput[0] << " "
          //             << camera_state.throughput[1] << " "
          //             << camera_state.throughput[2] << std::endl;

          //   std::cout << "query.GetContrib() : " << query.GetContrib()[0] <<
          //   " "
          //             << query.GetContrib()[1] << " " <<
          //             query.GetContrib()[2]
          //             << std::endl;

          color +=
              camera_state.throughput * vm_normalization_ * query.GetContrib();
        }

        if (!SampleScattering(camera_state, sampler, sp.bsdf, inct)) break;

        // std::cout << "camera_state.dVCM : " << camera_state.dVCM <<
        // std::endl;
      }

      const int res_x = static_cast<int>(film->Dimension().x());

      const int x = path_idx % res_x;
      const int y = path_idx / res_x;

      tile.AddSample(Vector2i{x, y}, color, 1.f);
      //   film->AddSplat(color,
      //                  Vector2f{static_cast<float>(x),
      //                  static_cast<float>(y)});
    }

    film->MergeTile(tile);
  }

  bool SampleScattering(SubPathState& light_state, Sampler* sampler,
                        const BSDF* bsdf, PrimitiveIntersection& inct) const {
    auto bsdf_sample_result = bsdf->SampleAll(
        -light_state.direction, TransMode::Radiance, sampler->Next3D());
    if (bsdf_sample_result.f.IsBlack()) return false;
    light_state.direction = bsdf_sample_result.dir;

    float bsdf_dir_pdf = bsdf_sample_result.pdf;
    float bsdf_rev_pdf = bsdf_sample_result.pdf;
    if (!bsdf_sample_result.is_delta)
      bsdf_rev_pdf =
          bsdf->PdfAll(-light_state.direction, bsdf_sample_result.dir);

    if (sampler->Next1D() > args_.cont_prob) return false;
    bsdf_dir_pdf *= args_.cont_prob;
    bsdf_rev_pdf *= args_.cont_prob;

    const float cos_theta_out =
        dot(inct.geometry_normal, bsdf_sample_result.dir);
    if (bsdf_sample_result.is_delta) {
      light_state.dVCM = 0.f;
      assert(bsdf_dir_pdf == bsdf_rev_pdf);
      light_state.dVC *= Mis(cos_theta_out);
      light_state.dVM *= Mis(cos_theta_out);

      light_state.specular_path &= 1;
    } else {
      light_state.dVC = Mis(cos_theta_out / bsdf_dir_pdf) *
                        (light_state.dVC * Mis(bsdf_rev_pdf) +
                         light_state.dVCM + mis_vm_weight_factor_);
      light_state.dVM = Mis(cos_theta_out / bsdf_dir_pdf) *
                        (light_state.dVM * Mis(bsdf_rev_pdf) +
                         light_state.dVCM * mis_vc_weight_factor_ + 1.f);

      light_state.dVCM = 1.f / bsdf_dir_pdf;

      light_state.specular_path &= 0;
    }

    light_state.origin = inct.pos;
    light_state.throughput *=
        bsdf_sample_result.f * (cos_theta_out / bsdf_dir_pdf);
    return true;
  }

  Spectrum ConnectVertices(const SubPathState& camera_state,
                           const PathVertex& light_vertex, Scene* scene,
                           const PrimitiveIntersection& inct,
                           const BSDF* bsdf) const {
    Vector3f direction = light_vertex.hit_point - inct.pos;
    const float dist2 = direction.dot();
    float distance = direction.length();
    direction = direction.normalized();

    const auto bsdf_eval =
        bsdf->EvalAll(direction, inct.wr, TransMode::Radiance);

    if (bsdf_eval.IsBlack()) return {};

    float camera_dir_pdf_w = bsdf->PdfAll(direction, inct.wr);
    float camera_rev_pdf_w = bsdf->PdfAll(inct.wr, direction);

    // if (camera_dir_pdf_w == 0.f || camera_rev_pdf_w == 0.f) return {};

    camera_dir_pdf_w *= args_.cont_prob;
    camera_rev_pdf_w *= args_.cont_prob;

    const auto light_eval = light_vertex.bsdf->EvalAll(
        -direction, light_vertex.in_dir, TransMode::Radiance);

    if (light_eval.IsBlack()) return {};

    float light_dir_pdf_w =
        light_vertex.bsdf->PdfAll(-direction, light_vertex.in_dir);
    float light_rev_pdf_w =
        light_vertex.bsdf->PdfAll(light_vertex.in_dir, -direction);

    // if (light_dir_pdf_w == 0.f || light_rev_pdf_w == 0.f) return {};

    light_dir_pdf_w *= args_.cont_prob;
    light_rev_pdf_w *= args_.cont_prob;

    float geometry_term = dot(light_vertex.normal, -direction) *
                          dot(inct.shading_normal, direction) / dist2;
    if (geometry_term < 0.f) return {};

    const float camera_dir_pdf_a =
        camera_dir_pdf_w * std::abs(dot(light_vertex.normal, -direction)) /
        dist2;
    const float light_dir_pdf_a =
        light_dir_pdf_w * std::abs(dot(inct.shading_normal, direction)) / dist2;

    const float w_light =
        Mis(camera_dir_pdf_a) * (mis_vm_weight_factor_ + light_vertex.dVCM +
                                 light_vertex.dVC * Mis(light_rev_pdf_w));

    const float w_camera =
        Mis(light_dir_pdf_a) * (mis_vm_weight_factor_ + camera_state.dVCM +
                                camera_state.dVC * Mis(camera_rev_pdf_w));

    const float mis_weight = 1.f / (w_light + 1.f + w_camera);

    const Spectrum contrib =
        (mis_weight * geometry_term) * bsdf_eval * light_eval;

    if (contrib.IsBlack()) {
      return {};
      //   const Ray shadow_ray{inct.pos, direction};
      //   if (scene->Occlude(shadow_ray)) return {};
    }

    const Ray shadow_ray{inct.pos, direction, Ray::Eps(), distance * 0.9f};
    if (scene->Occlude(shadow_ray)) return {};

    return contrib;
  }

  void ConnectToCamera(const SubPathState& light_state, Scene* scene,
                       Camera* camera, Film* film, Sampler* sampler,
                       const BSDF* bsdf, PrimitiveIntersection& inct) const {
    Vector3f dir_to_camera{};
    Vector2f image_pos{};
    if (!camera->ToRaster(sampler->Next2D(), inct.pos, &dir_to_camera,
                          &image_pos))
      return;
    const float dist_eye2 = dir_to_camera.dot();
    const float distance = dir_to_camera.length();
    dir_to_camera = dir_to_camera.normalized();

    Spectrum f = bsdf->Eval(dir_to_camera, inct.wr, TransMode::Radiance,
                            kBSDFAll & ~BSDFComponentType::eSpecular);
    if (f.IsBlack()) return;

    float pdf = bsdf->Pdf(dir_to_camera, inct.wr,
                          kBSDFAll & ~BSDFComponentType::eSpecular);
    float rev_pdf = bsdf->Pdf(inct.wr, dir_to_camera,
                              kBSDFAll & ~BSDFComponentType::eSpecular);
    // if (pdf == 0.f || rev_pdf == 0.f) return;

    rev_pdf *= args_.cont_prob;

    const float cos_to_cam = dot(inct.geometry_normal, dir_to_camera);
    const float image_to_surface_factor =
        camera->ImageAreaToSurfaceArea(dir_to_camera, cos_to_cam, distance);

    const float w_light = Mis(image_to_surface_factor / light_sub_path_count_) *
                          (mis_vm_weight_factor_ + light_state.dVCM +
                           light_state.dVC * Mis(rev_pdf));

    // std::cout << "w_light : " << w_light << std::endl;
    // std::cout << "image_to_surface_factor : " << image_to_surface_factor
    //           << std::endl;
    // std::cout << "light_sub_path_count_ : " << light_sub_path_count_
    //           << std::endl;
    // std::cout << "mis_vm_weight_factor_ : " << mis_vm_weight_factor_
    //           << std::endl;
    // std::cout << "light_state.dVCM : " << light_state.dVCM << std::endl;
    // std::cout << "light_state.dVC : " << light_state.dVC << std::endl;
    // std::cout << "rev_pdf : " << rev_pdf << std::endl;

    const float mis_weight = 1.f / (w_light + 1.f);

    const float surface_to_image_factor = 1.f / image_to_surface_factor;

    const Spectrum contrib = mis_weight * light_state.throughput * f /
                             (light_sub_path_count_ * surface_to_image_factor);

    if (!contrib.IsBlack()) {
      const Ray shadow_ray{inct.pos, dir_to_camera, Ray::Eps(),
                           distance * 0.9f};
      if (scene->Occlude(shadow_ray)) return;

      film->AddSplat(contrib, image_pos);
    }
  }

  Spectrum GetLightRadiance(Light* light) const {}

  Spectrum DirectIllumination(SubPathState& state,
                              const PrimitiveIntersection& inct,
                              const BSDF* bsdf, Scene* scene,
                              Sampler* sampler) const {
    auto [light, light_pick_pdf] = scene->SampleLight(sampler->Next1D());
    auto sample_result = light->Sample(inct.pos, sampler);

    if (sample_result.radiance.IsBlack()) return {};

    const Spectrum bsdf_eval =
        bsdf->EvalAll(sample_result.Ref2Light(), inct.wr, TransMode::Radiance);

    if (bsdf_eval.IsBlack()) return {};

    float bsdf_pdf_w = bsdf->PdfAll(sample_result.Ref2Light(), inct.wr);
    // if (bsdf_pdf_w == 0.f) return {};

    float bsdf_rev_pdf_w = bsdf->PdfAll(inct.wr, sample_result.Ref2Light());
    // if (bsdf_rev_pdf_w == 0.f) return {};

    bsdf_pdf_w *= light->IsDelta() ? 0.f : args_.cont_prob;
    bsdf_rev_pdf_w *= args_.cont_prob;

    const float w_light =
        Mis(bsdf_pdf_w / (light_pick_pdf * sample_result.pdf));

    float cos_to_light = dot(inct.shading_normal, sample_result.Ref2Light());
    float cos_at_light = dot(sample_result.normal, -sample_result.Ref2Light());

    const float w_camera =
        Mis(sample_result.emit_pdf * cos_to_light /
            (sample_result.pdf * cos_at_light)) *
        (mis_vm_weight_factor_ + state.dVCM + state.dVC * Mis(bsdf_rev_pdf_w));

    // std::cout << "w_camera : " << w_camera << std::endl;
    // std::cout << "sample_result.emit_pdf : " << sample_result.emit_pdf
    //           << std::endl;
    // std::cout << "cos_to_light : " << cos_to_light << std::endl;
    // std::cout << "sample_result.pdf : " << sample_result.pdf << std::endl;
    // std::cout << "cos_at_light : " << cos_at_light << std::endl;
    // std::cout << "mis_vm_weight_factor_ : " << mis_vm_weight_factor_
    //           << std::endl;
    // std::cout << " state.dVCM : " << state.dVCM << std::endl;
    // std::cout << "state.dVC : " << state.dVC << std::endl;
    // std::cout << "bsdf_rev_pdf_w : " << bsdf_rev_pdf_w << std::endl;

    const float mis_weight = 1.f / (w_light + 1.f + w_camera);

    const Spectrum contrib =
        (mis_weight * cos_to_light / (light_pick_pdf * sample_result.pdf)) *
        (sample_result.radiance * bsdf_eval);

    // std::cout << "contrib : " << contrib << std::endl;
    // std::cout << "mis_weight : " << mis_weight << std::endl;
    // std::cout << "w_camera : " << w_camera << std::endl;

    if (contrib.IsBlack()) {
      return {};
      //   const Ray shadow_ray{inct.pos, sample_result.Ref2Light()};
      //   if (scene->Occlude(shadow_ray)) return {};
    }

    const Ray shadow_ray{inct.pos, sample_result.Ref2Light(), Ray::Eps(),
                         (sample_result.pos - sample_result.ref).length()};
    if (scene->Occlude(shadow_ray)) return {};

    return contrib;
  }

  void GenerateLightSample(SubPathState& state, Scene* scene,
                           Sampler* sampler) {
    auto [light, light_pick_pdf] = scene->SampleLight(sampler->Next1D());
    auto sample_emit_result = light->SampleEmit(sampler);
    state.throughput =
        sample_emit_result.radiance *
        std::abs(dot(sample_emit_result.normal, sample_emit_result.dir));
    state.origin = sample_emit_result.pos;
    state.direction = sample_emit_result.dir;

    auto emission_pdf = sample_emit_result.pdf_pos *
                        sample_emit_result.pdf_dir * light_pick_pdf;

    state.throughput /= emission_pdf;
    state.path_length = 1;
    state.is_finite_light = light->IsFinite() ? 1 : 0;

    {
      state.dVCM = Mis(1.f / sample_emit_result.pdf_dir);
      if (!light->IsDelta()) {
        const float used_cos_light =
            light->IsFinite() ? std::abs(dot(sample_emit_result.normal,
                                             sample_emit_result.dir))
                              : 1.f;
        state.dVC = Mis(used_cos_light / emission_pdf);
      } else
        state.dVC = 0.f;

      state.dVM = state.dVC * mis_vc_weight_factor_;
    }
  }

  Vector2f GenerateCameraSample(const int pixel_index, SubPathState& state,
                                Camera* camera, Film* film, Sampler* sampler) {
    const int res_x = static_cast<int>(film->Dimension().x());

    const int x = pixel_index % res_x;
    const int y = pixel_index / res_x;

    const float u = x + sampler->Next1D();
    const float v = y + sampler->Next1D();
    const Ray primary_ray =
        camera->GenerateRay(Vector2f{u, v}, sampler->Next2D());

    const float camera_pdf = camera->ImageAreaToSolidAngle(primary_ray.d);

    state.origin = primary_ray.o;
    state.direction = primary_ray.d;
    state.throughput = Vector3f{1.f};

    state.path_length = 1;
    state.specular_path = 1;

    // state.dVCM = Mis(light_sub_path_count_ / camera_pdf);
    state.dVCM = Mis(1.f / camera_pdf);
    state.dVC = 0;
    state.dVM = 0;

    return Vector2f{u, v};
  }

  float Mis(float value) const { return value; }

  VCMRendererArgs args_;
  float light_sub_path_count_{};
  float mis_vm_weight_factor_{};
  float mis_vc_weight_factor_{};
  float vm_normalization_{};

  std::vector<PathVertex> light_vertices_;
  std::vector<int> path_ends_;

  HashGrid hash_grid_;
};

Rc<Renderer> CreateVCMRenderer(const VCMRendererArgs& args) {
  return RcNew<VCMRenderer>(args);
}

AJ_END
