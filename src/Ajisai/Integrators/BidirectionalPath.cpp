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

// #include <Ajisai/Core/BSDF.h>
#include <Ajisai/Core/Geometry.h>
#include <Ajisai/Integrators/Integrator.h>

#include <cstring>

using Ajisai::Core::CameraSamplingRecord;
using Ajisai::Core::Intersect;
using Ajisai::Core::LightSamplingRecord;
using Ajisai::Core::SurfaceInteraction;
using Ajisai::Core::VisibilityTester;

namespace Ajisai::Integrators {

template <typename T>
class ScopedAssignment {
  T* target = nullptr;
  T backup;

 public:
  ScopedAssignment() : target(nullptr), backup(T()) {}
  ScopedAssignment(T* target, T value) : target(target) {
    if (target) {
      backup = *target;
      *target = value;
    }
  }
  ~ScopedAssignment() {
    if (target) *target = backup;
  }
  ScopedAssignment(ScopedAssignment&&) = delete;
  ScopedAssignment(const ScopedAssignment&) = delete;
  ScopedAssignment& operator=(const ScopedAssignment&) = delete;
  ScopedAssignment& operator=(ScopedAssignment&& other) noexcept {
    if (target) *target = backup;
    target = other.target;
    backup = other.backup;
    other.target = nullptr;
    return *this;
  }
};

struct EndpointInteraction : Intersect {
  using Intersect::Intersect;
  //   Math::Vector3f p;
  union {
    const Core::Camera* camera;
    const Core::AreaLight* light;
  };
  EndpointInteraction() : light{nullptr} {}
  EndpointInteraction(const Core::Ray& ray)
      : Intersect(ray.Point(1)), light(nullptr) {}
  EndpointInteraction(const Core::Camera* camera, const Core::Ray& ray)
      : Intersect(ray.o), camera(camera) {}
  EndpointInteraction(const Core::AreaLight* light, const Core::Ray& ray)
      : Intersect(ray.o), light(light) {}
  // EndpointInteraction(const Core::Camera* camera, const Math::Vector3f& p)
  //     : Intersect(p), camera(camera) {}
  EndpointInteraction(const Core::AreaLight* light, const Core::Ray& r,
                      const Math::Vector3f& nl)
      : Intersect(r.o), light(light) {
    Ng = nl;
  }
};

enum class VertexType { Camera, Light, Surface };

enum class TransportMode { Radiance, Importance };

struct PathVertex {
  VertexType type;
  Math::Spectrum beta;
  union {
    EndpointInteraction ei;
    SurfaceInteraction si;
  };
  bool delta = false;
  float pdfFwd = 0, pdfRev = 0;
  PathVertex() : ei() {}
  PathVertex(VertexType type, const EndpointInteraction& ei,
             const Math::Spectrum& beta)
      : type(type), beta(beta), ei(ei) {}
  PathVertex(const SurfaceInteraction& si, const Math::Spectrum& beta)
      : type(VertexType::Surface), beta(beta), si(si) {}
  ~PathVertex() {}

  PathVertex(const PathVertex& v) { memcpy(this, &v, sizeof(PathVertex)); }
  PathVertex& operator=(const PathVertex& v) {
    memcpy(this, &v, sizeof(PathVertex));
    return *this;
  }

  const Intersect& GetInteraction() const {
    switch (type) {
      case VertexType::Surface:
        return si;
      default:
        return ei;
    }
  }
  const auto& p() const { return GetInteraction().p; }
  const auto& ng() const { return GetInteraction().Ng; }
  [[nodiscard]] Math::Vector3f Ns() const {
    if (type == VertexType::Surface) {
      return si.Ns;
    } else if (type == VertexType::Light || type == VertexType::Camera) {
      return ei.Ng;
    } else {
      return {};
    }
  }
  bool IsOnSurface() const { return ng() != Math::Vector3f{0.f}; }
  bool IsConnectible() const {
    switch (type) {
      case VertexType::Light:
        return true;
      case VertexType::Camera:
        return true;
      case VertexType::Surface:
        return !delta;
    }

    return false;  // NOTREACHED
  }

  float PdfLightOrigin(const Core::Scene& scene, const PathVertex& next) {
    const Core::AreaLight* light = ei.light;
    if (!light) {
      return 0.0f;
    }
    auto w = next.p() - p();
    w = w.normalized();
    float pdfPos = 0, pdfDir = 0;
    light->Pdf_Le(Core::Ray(p(), w), &pdfPos, &pdfDir);
    return scene.PdfLight(light) * pdfPos;
  }

  Math::Spectrum Le(const PathVertex& next) const {
    switch (type) {
      case VertexType::Surface: {
        auto wo = (next.p() - p()).normalized();
        return si.Le(wo);
      }
      case VertexType::Light: {
        auto* light = ei.light;
        auto wo = (next.p() - p()).normalized();
        return light->Li(wo);
      }
      case VertexType::Camera:
      default:
        return Math::Spectrum{0.f};
    }
  }

  float Pdf(const Core::Scene& scene, const PathVertex* prev,
            const PathVertex& next) {
    if (type == VertexType::Light) {
      return PdfLight(scene, next);
    }
    auto wn = next.p() - p();
    if (Math::dot(wn, wn) == 0) return 0;
    wn = wn.normalized();
    Math::Vector3f wp;
    if (prev) {
      wp = prev->p() - p();
      if (Math::dot(wp, wp) == 0) return 0;
      wp = wp.normalized();
    } else {
      assert(type == VertexType::Camera);
    }
    float pdf = 0;
    if (type == VertexType::Surface) {
      auto wo = si.bsdf->toLocal(-wp);
      auto wi = si.bsdf->toLocal(wn);
      pdf = si.bsdf->EvaluatePdf(wo, wi);
    } else if (type == VertexType::Camera) {
      auto* camera = ei.camera;
      float _;
      camera->Pdf_We(ei.SpawnRay(wn), &_, &pdf);
    } else {
      std::exit(1);
    }
    return ConvertDensity(pdf, next);
  }

  float PdfLight(const Core::Scene& scene, const PathVertex& next) {
    const Core::AreaLight* light = ei.light;

    if (!light) {
      return 0;
    }
    auto w = next.p() - p();

    float invDist2 = 1 / Math::dot(w, w);
    w = w.normalized();

    float pdf;
    float pdfPos = 0, pdfDir = 0;
    light->Pdf_Le(Core::Ray(p(), w), &pdfPos, &pdfDir);
    pdf = pdfDir * invDist2;
    if (next.IsOnSurface()) {
      pdf *= abs(Math::dot(next.ng(), w));
    }
    return pdf;
  }

  Math::Spectrum f(const PathVertex& next) const {
    auto wi = (next.p() - p()).normalized();
    switch (type) {
      case VertexType::Surface: {
        return si.bsdf->Evaluate(si.wo, wi);
      }
      default:
        return {};
    }
  }

  float ConvertDensity(float pdf, const PathVertex& next) const {
    // Return solid angle density if _next_ is an infinite area light
    // if (next.IsInfiniteLight()) return pdf;
    auto w = next.p() - p();
    if (Math::dot(w, w) == 0) return 0;
    float invDist2 = 1 / Math::dot(w, w);
    if (next.IsOnSurface())
      pdf *= std::abs(Math::dot(next.ng(), w * std::sqrt(invDist2)));
    return pdf * invDist2;
  }

  static inline PathVertex CreateCamera(const Core::Camera* camera,
                                        const Core::Ray& ray,
                                        const Math::Spectrum& beta);
  static inline PathVertex CreateLight(const EndpointInteraction& ei,
                                       const Math::Spectrum& beta, float pdf);
  static inline PathVertex CreateSurface(const SurfaceInteraction& si,
                                         const Math::Spectrum& beta, float pdf,
                                         const PathVertex& prev);
  static inline PathVertex CreateLight(const Core::AreaLight* light,
                                       const Core::Ray& ray,
                                       const Math::Vector3f& N,
                                       const Math::Spectrum& Le, float pdf);
  // static inline PathVertex CreateCamera(const Core::Camera* camera,
  //                                       const Math::Vector3f& p,
  //                                       const Math::Spectrum& beta);

  static Math::Spectrum G(const Core::Scene& scene, const PathVertex& v0,
                          const PathVertex& v1) {
    auto d = v0.p() - v1.p();
    float g = 1 / Math::dot(d, d);
    // d *= std::sqrt(g);
    d = d.normalized();
    if (v0.IsOnSurface()) g *= abs(Math::dot(v0.Ns(), d));
    if (v1.IsOnSurface()) g *= abs(Math::dot(v1.Ns(), d));
    VisibilityTester vis(v0.GetInteraction(), v1.GetInteraction());
    return g * vis.Tr(scene);
  }
};

inline PathVertex PathVertex::CreateCamera(const Core::Camera* camera,
                                           const Core::Ray& ray,
                                           const Math::Spectrum& beta) {
  return PathVertex(VertexType::Camera, EndpointInteraction(camera, ray), beta);
}

inline PathVertex PathVertex::CreateLight(const EndpointInteraction& ei,
                                          const Math::Spectrum& beta,
                                          float pdf) {
  PathVertex v(VertexType::Light, ei, beta);
  v.pdfFwd = pdf;
  return v;
}

inline PathVertex PathVertex::CreateSurface(const SurfaceInteraction& si,
                                            const Math::Spectrum& beta,
                                            float pdf, const PathVertex& prev) {
  PathVertex v(si, beta);
  v.pdfFwd = prev.ConvertDensity(pdf, v);
  return v;
}

inline PathVertex PathVertex::CreateLight(const Core::AreaLight* light,
                                          const Core::Ray& ray,
                                          const Math::Vector3f& N,
                                          const Math::Spectrum& Le, float pdf) {
  PathVertex v(VertexType::Light, EndpointInteraction(light, ray, N), Le);
  v.pdfFwd = pdf;
  return v;
}

// inline PathVertex PathVertex::CreateCamera(const Core::Camera* camera,
//                                            const Math::Vector3f& p,
//                                            const Math::Spectrum& beta) {
//   return PathVertex(VertexType::Camera, EndpointInteraction(camera, p),
//   beta);
// }

class BDPTIntegrator : public Integrator {
 public:
  explicit BDPTIntegrator(PluginManager::AbstractManager& manager,
                          const std::string plugin)
      : Integrator{manager, plugin} {}

  size_t RandomWalk(const Core::Scene& scene, Core::Sampler& sampler,
                    Core::Ray& ray, Math::Spectrum& beta, float pdf,
                    size_t depth, TransportMode mode, PathVertex* path) const {
    using Core::BSDFSamplingRecord;
    using Core::BSDFType;
    if (depth == 0) return 0;

    int bounces = 0;
    float pdfFwd = pdf, pdfRev = 0.f;
    while (true) {
      Core::Intersection isect;
      bool foundIntersection = scene.Intersect(ray, &isect);
      if (beta.isBlack()) break;
      auto& vertex = path[bounces];
      auto& prev = path[bounces - 1];
      if (!foundIntersection) {
        // if (mode == TransportMode::Radiance) {
        //   vertex =
        //       PathVertex::CreateLight(EndpointInteraction(ray), beta,
        //       pdfFwd);
        //   ++bounces;
        // }
        break;
      }

      Core::Triangle triangle{};
      isect.mesh->GetTriangle(isect.triId, &triangle);
      auto p = ray.Point(isect.t);
      SurfaceInteraction si(-ray.d, p, triangle, isect);
      isect.mesh->GetMaterial()->ComputeScatteringFunction(&si);

      vertex = PathVertex::CreateSurface(si, beta, pdfFwd, prev);
      if (++bounces >= depth) break;
      Math::Vector3f wi, wo = si.wo;

      BSDFSamplingRecord bRec(si, sampler.Next2D());
      si.bsdf->Sample(bRec);
      if (bRec.pdf <= 0.f) break;
      pdfFwd = bRec.pdf;
      wi = si.bsdf->toWorld(bRec.wi);
      beta *= bRec.f * std::abs(Math::dot(wi, si.Ns)) / bRec.pdf;
      pdfRev = si.bsdf->EvaluatePdf(bRec.wi, bRec.wo);
      if (bRec.type & BSDFType::BSDF_SPECULAR) {
        vertex.delta = true;
        pdfFwd = pdfRev = 0.f;
      }
      ray = si.SpawnRay(wi);
      prev.pdfRev = vertex.ConvertDensity(pdfRev, prev);
    }

    return bounces;
  }

  size_t GenerateCameraSubpath(const Core::Scene& scene,
                               const Core::Camera& camera,
                               const Math::Vector2i& raster,
                               Core::Sampler& sampler, size_t depth,
                               PathVertex* path) const {
    if (depth == 0) return 0;
    const float u =
        (raster.x() + sampler.Next1D()) / camera.GetFilm()->Dimension().x();
    const float v =
        (raster.y() + sampler.Next1D()) / camera.GetFilm()->Dimension().y();
    auto ray = camera.GenerateRay(u, v);
    auto beta = Math::Spectrum(1);

    float pdfPos, pdfDir;
    path[0] = PathVertex::CreateCamera(&camera, ray, beta);
    camera.Pdf_We(ray, &pdfPos, &pdfDir);
    if (pdfDir <= 0 || pdfPos <= 0) {
      return 0;
    }
    return RandomWalk(scene, sampler, ray, beta, pdfDir, depth - 1,
                      TransportMode::Radiance, path + 1) +
           1;
  }

  size_t GenerateLightSubpath(const Core::Scene& scene, Core::Sampler& sampler,
                              size_t depth, PathVertex* path) const {
    if (depth == 0) return 0;

    float lightPdf = 0.f;
    auto sampleLight = scene.SampleOneLight(sampler.Next1D(), &lightPdf);
    Core::Ray ray;
    Math::Vector3f nLight;
    float pdfPos, pdfDir;
    Math::Spectrum Le;
    sampleLight->Sample_Le(sampler.Next2D(), sampler.Next2D(), &ray, nLight,
                           &pdfPos, &pdfDir, Le);
    if (lightPdf <= 0 || pdfPos <= 0 || pdfDir <= 0 || Le.isBlack()) {
      return 0;
    }

    path[0] = PathVertex::CreateLight(sampleLight, ray, nLight, Le,
                                      pdfPos * lightPdf);
    Math::Spectrum beta =
        Le * abs(Math::dot(nLight, ray.d)) / (lightPdf * pdfPos * pdfDir);
    return 1 + RandomWalk(scene, sampler, ray, beta, pdfDir, depth - 1,
                          TransportMode::Importance, path + 1);
  }

  template <int Power>
  float MisWeight(const Core::Scene& scene, Core::Sampler& sampler,
                  PathVertex* cameraVertices, size_t t,
                  PathVertex* lightVertices, size_t s,
                  PathVertex& sampled) const {
    if (s + t == 2) return 1;
    auto remap0 = [](float x) { return x != 0 ? std::pow(x, Power) : 1.0f; };
    (void)remap0;
    float sumRi = 0;

    // p_0 ... pt  qs ... q_0
    auto* pt = t > 0 ? &cameraVertices[t - 1] : nullptr;
    auto* qs = s > 0 ? &lightVertices[s - 1] : nullptr;

    auto* ptMinus = t > 1 ? &cameraVertices[t - 2] : nullptr;
    auto* qsMinus = s > 1 ? &lightVertices[s - 2] : nullptr;

    ScopedAssignment<PathVertex> _a1;
    if (s == 1)
      _a1 = {qs, sampled};
    else if (t == 1)
      _a1 = {pt, sampled};
    //        if(s == 1){
    //            printf("b %f\n",lightPath[s-1].pdfFwd);
    //        }
    ScopedAssignment<bool> _a2, _a3;
    if (pt) _a2 = {&pt->delta, false};
    if (qs) _a3 = {&qs->delta, false};

    // now connect pt to qs

    // we need to compute pt->pdfRev
    // segfault ?
    ScopedAssignment<float> _a4;
    if (pt) {
      float pdfRev;
      if (s > 0) {
        pdfRev = qs->Pdf(scene, qsMinus, *pt);
      } else {
        pdfRev = pt->PdfLightOrigin(scene, *ptMinus);
      }
      _a4 = {&pt->pdfRev, pdfRev};
    }

    // now ptMinus->pdfRev
    ScopedAssignment<float> _a5;
    if (ptMinus) {
      float pdfRev;
      if (s > 0) {
        pdfRev = pt->Pdf(scene, qs, *ptMinus);
      } else {
        pdfRev = pt->PdfLight(scene, *ptMinus);
      }
      _a5 = {&ptMinus->pdfRev, pdfRev};
    }

    // now qs
    ScopedAssignment<float> _a6;
    if (qs) {
      _a6 = {&qs->pdfRev, pt->Pdf(scene, ptMinus, *qs)};
    }
    //        printf("%f\n",sampled.pdfFwd);
    // now qsMinus
    ScopedAssignment<float> _a7;
    if (qsMinus) {
      _a7 = {&qsMinus->pdfRev, qs->Pdf(scene, pt, *qsMinus)};
    }
    float ri = 1;
    for (int i = (int)t - 1; i > 0; i--) {
      ri *= remap0(cameraVertices[i].pdfRev) / remap0(cameraVertices[i].pdfFwd);
      if (!cameraVertices[i].delta && !cameraVertices[i - 1].delta) {
        sumRi += ri;
      }
    }
    ri = 1;
    for (int i = (int)s - 1; i >= 0; i--) {
      ri *= remap0(lightVertices[i].pdfRev) / remap0(lightVertices[i].pdfFwd);
      bool delta = i > 0 ? lightVertices[i - 1].delta
                         : false /*lightVertices[i].IsDeltaLight()*/;
      if (!lightVertices[i].delta && !delta) {
        sumRi += ri;
      }
    }
    return 1.0 / (1.0 + sumRi);
  }

  Math::Spectrum ConnectPath(const Core::Scene& scene, Core::Sampler& sampler,
                             PathVertex* cameraVertices, size_t t,
                             PathVertex* lightVertices, size_t s,
                             Math::Vector2f* pRaster) const {
    if (t > 1 && s != 0 && cameraVertices[t - 1].type == VertexType::Light)
      return Math::Spectrum(0.f);
    Math::Spectrum L(0.f);
    PathVertex sampled{};
    if (s == 0) {
      const PathVertex& pt = cameraVertices[t - 1];
      L = pt.Le(cameraVertices[t - 2]) * pt.beta;
    } else if (t == 1) {
      const PathVertex& qs = lightVertices[s - 1];
      auto camera = cameraVertices[0].ei.camera;
      if (qs.IsConnectible()) {
        VisibilityTester vis;
        CameraSamplingRecord cRec;
        camera->Sample_Wi(sampler.Next2D(), qs.GetInteraction(), &cRec, &vis);
        *pRaster = cRec.pos;
        if (cRec.pdf > 0 && !cRec.I.isBlack()) {
          // Initialize dynamically sampled vertex and _L_ for $t=1$ case
          sampled = PathVertex::CreateCamera(camera, vis.shadowRay,
                                             cRec.I / cRec.pdf);
          L = qs.beta * qs.f(sampled) * sampled.beta;
          if (qs.IsOnSurface()) L *= abs(Math::dot(cRec.wi, qs.Ns()));
          // Only check visibility after we know that the path would
          // make a non-zero contribution.
          if (!L.isBlack()) L *= vis.Tr(scene);
        }
      }
    } else if (s == 1) {
      // Sample a point on a light and connect it to the camera subpath
      const PathVertex& pt = cameraVertices[t - 1];
      auto& lightVertex = lightVertices[0];
      if (pt.IsConnectible()) {
        float lightPdf;
        VisibilityTester vis;
        LightSamplingRecord lRec;
        lightVertex.ei.light->Sample_Li(sampler.Next2D(), pt.GetInteraction(),
                                        lRec, vis);
        const Core::AreaLight* light = lightVertex.ei.light;
        if (lRec.pdf > 0 && !lRec.Li.isBlack()) {
          EndpointInteraction ei(light, vis.shadowRay);
          sampled = PathVertex::CreateLight(
              ei, lRec.Li / (scene.PdfLight(light) * lRec.pdf), 0);
          sampled.pdfFwd = sampled.PdfLightOrigin(scene, pt);
          L = pt.beta * pt.f(sampled) * sampled.beta;
          if (pt.IsOnSurface()) L *= abs(Math::dot(lRec.wi, pt.Ns()));
          // Only check visibility if the path would carry radiance.
          if (!L.isBlack()) L *= vis.Tr(scene);
        }
      }
    } else {
      // Handle all other bidirectional connection cases
      const PathVertex &qs = lightVertices[s - 1], &pt = cameraVertices[t - 1];
      if (qs.IsConnectible() && pt.IsConnectible()) {
        L = qs.beta * qs.f(pt) * pt.f(qs) * pt.beta;
        // VLOG(2) << "General connect s: " << s << ", t: " << t << " qs: " <<
        // qs
        //         << ", pt: " << pt
        //         << ", qs.f(pt): " << qs.f(pt, TransportMode::Importance)
        //         << ", pt.f(qs): " << pt.f(qs, TransportMode::Radiance)
        //         << ", G: " << G(scene, sampler, qs, pt)
        //         << ", dist^2: " << DistanceSquared(qs.p(), pt.p());
        if (!L.isBlack()) L *= PathVertex::G(scene, qs, pt);
      }
    }

    if (L.isBlack()) return {};
    float misWeight = 1.0f / (s + t);
    misWeight = MisWeight<1>(scene, sampler, cameraVertices, t, lightVertices,
                             s, sampled);
    assert(misWeight >= 0);
    L *= misWeight;
    return L.removeNaN();
  }

  virtual Math::Spectrum Li(Core::Scene* scene, Core::Camera* camera,
                            const Math::Vector2i& raster,
                            Core::Sampler* sampler) const override {
    PathVertex* cameraVertices = new PathVertex[maxDepth + 2];
    PathVertex* lightVertices = new PathVertex[maxDepth + 1];

    size_t nCamera = GenerateCameraSubpath(*scene, *camera, raster, *sampler,
                                           maxDepth + 2, cameraVertices);
    size_t nLight =
        GenerateLightSubpath(*scene, *sampler, maxDepth + 1, lightVertices);

    Math::Spectrum L(0.f);
    for (size_t t = 1; t <= nCamera; ++t) {
      for (size_t s = 0; s <= nLight; ++s) {
        int depth = int(t + s) - 2;
        if ((s == 1 && t == 1) || depth < 0 || depth > maxDepth) continue;
        Math::Vector2f pRaster{(float)raster.x(), (float)raster.y()};
        Math::Spectrum LPath = ConnectPath(*scene, *sampler, cameraVertices, t,
                                           lightVertices, s, &pRaster);
        if (t != 1) {
          // if (visualizeMIS) {
          //   pyramid.at(BufferIndex(s, t))->AddSplat(LPath, raster);
          // }
          L += LPath;
        } else {
          // if (visualizeMIS) {
          //   pyramid.at(BufferIndex(s, t))->AddSplat(LPath, pRaster);
          // }
          camera->GetFilm()->AddSplat(LPath, pRaster);
          // film->AddSplat(LPath, pRaster);
        }
      }
    }

    delete[] lightVertices;
    delete[] cameraVertices;
    return L;
  }

 private:
  int rrDepth = 5, maxDepth = 16;
};

}  // namespace Ajisai::Integrators

AJISAI_PLUGIN_REGISTER(BDPTIntegrator, Ajisai::Integrators::BDPTIntegrator,
                       "ajisai.integrators.Integrator/0.0.1")