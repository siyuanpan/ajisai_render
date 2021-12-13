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

// using Ajisai::Core::CameraSamplingRecord;
// using Ajisai::Core::Intersect;
// using Ajisai::Core::LightSamplingRecord;
// using Ajisai::Core::SurfaceInteraction;
// using Ajisai::Core::VisibilityTester;
using namespace Ajisai::Math;
using namespace Ajisai::Core;

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
      pdf *= std::abs(Math::dot(next.ng(), w));
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
    w = w.normalized();
    if (next.IsOnSurface()) pdf *= std::abs(Math::dot(next.ng(), w));
    // pdf *= std::abs(Math::dot(next.ng(), w * std::sqrt(invDist2)));
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
    if (v0.IsOnSurface()) g *= std::abs(Math::dot(v0.Ns(), d));
    if (v1.IsOnSurface()) g *= std::abs(Math::dot(v1.Ns(), d));
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

  struct Vertex {
    Spectrum throughput;
    uint32_t length;

    SurfaceInteraction si;
    Vector3f inDir;

    float DVCM;
    float DVC;
  };

  struct PathState {
    Vector3f origin;
    Vector3f direction;
    Spectrum throughput;
    uint PathLength : 30;
    bool isFiniteLight : 1;
    bool SpecularPath : 1;

    float DVCM;
    float DVC;
  };

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
                      TransportMode::Importance, path + 1) +
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
        Le * std::abs(Math::dot(nLight, ray.d)) / (lightPdf * pdfPos * pdfDir);
    return 1 + RandomWalk(scene, sampler, ray, beta, pdfDir, depth - 1,
                          TransportMode::Radiance, path + 1);
  }

  template <int Power>
  float MisWeight(const Core::Scene& scene, Core::Sampler& sampler,
                  PathVertex* cameraVertices, size_t t,
                  PathVertex* lightVertices, size_t s,
                  PathVertex& sampled) const {
    if (s + t == 2) return 1;
    auto remap0 = [](float x) -> float {
      return x != 0 ? std::pow(x, Power) : 1.0f;
    };
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
    // if (s == 1) {
    //   printf("b %f\n", lightVertices[s - 1].pdfFwd);
    // }
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
      // if (s != 0)
      //   printf("before rev %f s %d pdfRev %f\n", pt->pdfRev, (int)s, pdfRev);
      _a4 = {&pt->pdfRev, pdfRev};
    }
    // if (pt) printf("after rev %f\n", pt->pdfRev);

    // now ptMinus->pdfRev
    ScopedAssignment<float> _a5;
    // if (ptMinus) printf("before rev %f\n", ptMinus->pdfRev);
    if (ptMinus) {
      float pdfRev;
      if (s > 0) {
        pdfRev = pt->Pdf(scene, qs, *ptMinus);
      } else {
        pdfRev = pt->PdfLight(scene, *ptMinus);
      }
      _a5 = {&ptMinus->pdfRev, pdfRev};
    }
    // if (ptMinus) printf("after rev %f\n", ptMinus->pdfRev);

    // now qs
    ScopedAssignment<float> _a6;
    if (qs) {
      _a6 = {&qs->pdfRev, pt->Pdf(scene, ptMinus, *qs)};
    }
    // printf("%f\n", sampled.pdfFwd);
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
          if (qs.IsOnSurface()) L *= std::abs(Math::dot(cRec.wi, qs.Ns()));
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
          if (pt.IsOnSurface()) L *= std::abs(Math::dot(lRec.wi, pt.Ns()));
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

    // if (s == 1) printf("t %d s %d\n", (int)t, (int)s);
    misWeight = MisWeight<1>(scene, sampler, cameraVertices, t, lightVertices,
                             s, sampled);
    assert(misWeight >= 0);
    L *= misWeight;
    return L.removeNaN();
  }

  template <int pow>
  float MIS(float fVal) const {
    // Use power heuristic
    return std::pow(fVal, pow);
  }

  PathState SampleLightSource(Scene* scene, Sampler* sampler) const {
    PathState ret;

    float lightPdf = 0.f;
    auto sampleLight = scene->SampleOneLight(sampler->Next1D(), &lightPdf);
    // printf("lightPdf %f\n", lightPdf);

    Ray lightRay;
    Vector3f nLight;
    float posPdf, dirPdf;

    sampleLight->Sample_Le(sampler->Next2D(), sampler->Next2D(), &lightRay,
                           nLight, &posPdf, &dirPdf, ret.throughput);
    // printf(
    //     "ray.o (%f %f %f) ray.d (%f %f %f) nLight (%f %f %f) posPdf (%f) "
    //     "dirPdf(%f) E (%f %f %f)\n",
    //     lightRay.o[0], lightRay.o[1], lightRay.o[2], lightRay.d[0],
    //     lightRay.d[1], lightRay.d[2], nLight.x(), nLight.y(), nLight.z(),
    //     posPdf, dirPdf, ret.throughput[0], ret.throughput[1],
    //     ret.throughput[2]);
    if ((posPdf * dirPdf) == 0.0f) return ret;

    // dirPdf *= lightPdf;
    // posPdf *= lightPdf;
    // printf("dirPdf %f, posPdf %f, lightPdf %f emitPdf %f\n", dirPdf, posPdf,
    //        lightPdf, dirPdf * posPdf * lightPdf);
    float emitPdf = dirPdf * posPdf * lightPdf;
    ret.throughput /= emitPdf;
    ret.isFiniteLight = sampleLight->isFinite();
    ret.SpecularPath = true;
    ret.PathLength = 1;
    ret.direction = lightRay.d.normalized();
    ret.origin = lightRay.o;

    float emitCos = dot(nLight, lightRay.d);
    ret.DVCM = MIS<pow>((posPdf * lightPdf) / emitPdf);
    ret.DVC = sampleLight->isDelta()
                  ? 0.f
                  : (sampleLight->isFinite()
                         ? MIS<pow>(emitCos / (dirPdf * posPdf * lightPdf))
                         : MIS<pow>(1.f / (dirPdf * posPdf * lightPdf)));
    // printf("dirPdf %f, posPdf %f, lightPdf %f emitPdf %f\n", dirPdf, posPdf,
    //        lightPdf, emitPdf);
    // printf("E (%f %f %f), emitCos %f, DVCM %f DVC %f\n", ret.throughput[0],
    //        ret.throughput[1], ret.throughput[2], emitCos, ret.DVCM, ret.DVC);

    return ret;
  }

  Spectrum ConnectToCamera(Scene* scene, Camera* camera, Sampler* sampler,
                           SurfaceInteraction& si, Vertex& lightVertex,
                           Vector2f& pRaster) const {
    Vector3f dirToCamera;
    // Vector2f pRaster;
    if (!camera->ToRaster(sampler->Next2D(), si, dirToCamera, pRaster))
      return {};

    float distToCamera = dirToCamera.length();
    dirToCamera = dirToCamera.normalized();

    // printf("(%f %f %f) (%f %f)\n", lightVertex.throughput[0],
    //        lightVertex.throughput[1], lightVertex.throughput[2], pRaster[0],
    //        pRaster[1]);

    Spectrum f = si.bsdf->Evaluate(dirToCamera, lightVertex.inDir) *
                 std::abs(dot(lightVertex.inDir, si.Ns)) /
                 std::abs(dot(lightVertex.inDir, si.Ng));
    // printf("(%f %f %f) (%f %f %f) %f %f (%f %f %f)\n", dirToCamera[0],
    //        dirToCamera[1], dirToCamera[2],
    //        si.bsdf->Evaluate(dirToCamera, lightVertex.inDir)[0],
    //        si.bsdf->Evaluate(dirToCamera, lightVertex.inDir)[1],
    //        si.bsdf->Evaluate(dirToCamera, lightVertex.inDir)[2],
    //        std::abs(dot(lightVertex.inDir, si.Ns)),
    //        std::abs(dot(lightVertex.inDir, si.Ng)), f[0], f[1], f[2]);
    if (f.isBlack()) return {};
    // printf("(%f %f %f)\n", f[0], f[1], f[2]);
    float pdf = si.bsdf->EvaluatePdf(si.bsdf->toLocal(lightVertex.inDir),
                                     si.bsdf->toLocal(dirToCamera));
    float revPdf = si.bsdf->EvaluatePdf(si.bsdf->toLocal(dirToCamera),
                                        si.bsdf->toLocal(lightVertex.inDir));
    // printf("%f %f\n", pdf, revPdf);
    if (pdf == 0.f || revPdf == 0.f) return {};

    float cosToCam = dot(si.Ng, dirToCamera);

    CameraSamplingRecord cRec;
    VisibilityTester tester;
    camera->Sample_Wi(sampler->Next2D(), si, &cRec, &tester);
    (void)tester;
    // printf("%f\n", cRec.pdf);

    float cameraPdfA =
        cRec.pdf * std::abs(cosToCam) / (distToCamera * distToCamera);
    // printf("%d\n", camera->GetPixelCount());
    float WLight =
        MIS<pow>(cameraPdfA /
                 camera->GetPixelCount()  //(float)camera->GetPixelCount()
                 ) *
        (lightVertex.DVCM + lightVertex.DVC * MIS<pow>(revPdf));
    float MISWeight = 1.f / (WLight + 1.f);
    // printf("%f\n", MISWeight);
    Spectrum contrib = MISWeight * lightVertex.throughput * f * cameraPdfA /
                       camera->GetPixelCount();
    //  (float)camera->GetPixelCount();
    // printf("%f %f %f\n", contrib[0], contrib[1], contrib[2]);
    // printf("(%f %f %f) MISWeight %f f (%f %f %f) cameraPdfA %f A %f\n",
    //        lightVertex.throughput[0], lightVertex.throughput[1],
    //        lightVertex.throughput[2], MISWeight, f[0], f[1], f[2],
    //        cameraPdfA, camera->A());

    Ray rayToCam =
        Ray(si.p, dirToCamera, Ray::Eps(), distToCamera * (1.f - Ray::Eps()));

    if (!contrib.isBlack() && !scene->Occlude(rayToCam)) {
      // printf("%f %f %f\n", contrib[0], contrib[1], contrib[2]);
      return contrib;
    }
    return {};
  }

  int GenerateLightPath(Scene* scene, Sampler* sampler, const int maxDepth,
                        Vertex* lightVertices, Camera* camera,
                        int* vertexCount) const {
    if (maxDepth == 0) {
      *vertexCount = 0;
      return 0;
    }

    PathState lightPathState = SampleLightSource(scene, sampler);
    if (lightPathState.throughput.isBlack()) return 0;
    // printf("(%f %f %f) \n", lightPathState.throughput[0],
    //        lightPathState.throughput[1], lightPathState.throughput[2]);

    if (lightPathState.PathLength >= maxDepth) {
      *vertexCount = 0;
      return lightPathState.PathLength;
    }

    *vertexCount = 0;
    while (true) {
      Ray pathRay(lightPathState.origin, lightPathState.direction);
      // printf("(%f %f %f) (%f %f %f)\n", pathRay.o[0], pathRay.o[1],
      //        pathRay.o[2], pathRay.d[0], pathRay.d[1], pathRay.d[2]);
      Intersection isect;
      if (!scene->Intersect(pathRay, &isect)) {
        return lightPathState.PathLength;
      }

      if (lightPathState.PathLength > 1 || lightPathState.isFiniteLight) {
        // printf("1 %f %f\n", lightPathState.DVCM, isect.t);
        lightPathState.DVCM *= MIS<pow>(isect.t * isect.t);
        // printf("(%f) (%f)\n", (pathRay.Point(isect.t) - pathRay.o).length(),
        //        isect.t);
        // printf("2 %f\n", lightPathState.DVCM);
      }

      float cosIn = std::abs(dot(isect.Ng, -pathRay.d));
      lightPathState.DVCM /= MIS<pow>(cosIn);
      lightPathState.DVC /= MIS<pow>(cosIn);
      // printf("cosIn %f DVCM %f DVC %f\n", cosIn, lightPathState.DVCM,
      //        lightPathState.DVC);

      Triangle triangle{};
      isect.mesh->GetTriangle(isect.triId, &triangle);
      auto p = pathRay.Point(isect.t);
      SurfaceInteraction si(-pathRay.d, p, triangle, isect);
      isect.mesh->GetMaterial()->ComputeScatteringFunction(&si);

      BSDFSamplingRecord bRec(si, sampler->Next2D());
      si.bsdf->Sample(bRec);
      // if (bRec.pdf <= 0.f) break;

      auto specular = bRec.type & BSDFType::BSDF_SPECULAR;
      if (!specular) {
        Vertex& lightVertex = lightVertices[(*vertexCount)++];
        lightVertex.throughput = lightPathState.throughput;
        lightVertex.length = lightPathState.PathLength + 1;
        lightVertex.si = si;
        lightVertex.inDir = -lightPathState.direction;
        lightVertex.DVCM = lightPathState.DVCM;
        lightVertex.DVC = lightPathState.DVC;

        // printf("(%f %f %f) \n", lightVertex.throughput[0],
        //        lightVertex.throughput[1], lightVertex.throughput[2]);

        // connect to camera
        Vector2f ptRaster;
        Spectrum connectRadiance =
            ConnectToCamera(scene, camera, sampler, si, lightVertex, ptRaster);

        if (connectRadiance.isBlack()) break;

        // printf("(%f %f %f) (%f %f)\n", connectRadiance[0],
        // connectRadiance[1],
        //        connectRadiance[2], ptRaster[0], ptRaster[1]);

        camera->GetFilm()->AddSplat(connectRadiance, ptRaster);
      }

      if (++lightPathState.PathLength >= maxDepth) break;

      // if (!SampleScattering()) break;

      Spectrum f = bRec.f;
      Vector3f wi = bRec.wi;
      float scatteredPdf = bRec.pdf;

      float revPdf = si.bsdf->EvaluatePdf(wi, si.bsdf->toLocal(-pathRay.d));

      if (f.isBlack() || scatteredPdf == 0.f) break;
      // printf("(%f %f %f) %f\n", f[0], f[1], f[2], scatteredPdf);

      if (!specular && rrDepth != -1 && lightPathState.PathLength > rrDepth) {
        float q = std::min(0.95f, lightPathState.throughput.max());
        if (sampler->Next1D() >= q) break;
        lightPathState.throughput /= q;
      }

      lightPathState.origin = si.p;
      lightPathState.direction = si.bsdf->toWorld(bRec.wi);

      float cosOut = std::abs(dot(si.Ns, si.bsdf->toWorld(bRec.wi)));
      if (!specular) {
        lightPathState.SpecularPath &= 0;

        lightPathState.DVCM =
            MIS<pow>(cosOut / scatteredPdf) *
            (lightPathState.DVC * MIS<pow>(revPdf) + lightPathState.DVCM);
        lightPathState.DVC = MIS<pow>(1.0f / scatteredPdf);
        // printf("%f %f\n", lightPathState.DVCM, lightPathState.DVC);
      } else {
        lightPathState.SpecularPath &= 1;

        lightPathState.DVCM = 0.f;
        lightPathState.DVC *= MIS<pow>(cosOut);
        // printf("%f %f\n", lightPathState.DVCM, lightPathState.DVC);
      }

      lightPathState.throughput *= f * cosOut / scatteredPdf;
      // printf("(%f %f %f) \n", lightPathState.throughput[0],
      //        lightPathState.throughput[1], lightPathState.throughput[2]);
    }

    return lightPathState.PathLength;
  }

  // bool SampleScattering() const {}

  void SampleCamera(Camera* camera, Ray& ray, Sampler* sampler,
                    PathState& initPathState) const {
    // CameraSamplingRecord cRec;
    // VisibilityTester tester;
    // SurfaceInteraction si;
    // ray.d = ray.d.normalized();
    // si.p = ray.Point(camera->GetFocusDistance());
    // camera->Sample_Wi(sampler->Next2D(), si, &cRec, &tester);
    // (void)tester;
    // float cosAtCam = Math::Dot(pCamera->mDir, primRay.mDir);
    // float rasterToCamDist = pCamera->GetImagePlaneDistance() / cosAtCam;
    // float cameraPdfW = rasterToCamDist * rasterToCamDist / cosAtCam;
    float virtualImagePlaneDistance = camera->GetImagePlaneDist();
    float cosThetaCamera = dot(camera->GetDir(), ray.d.normalized());
    float imagePointToCameraDistance =
        virtualImagePlaneDistance / cosThetaCamera;
    float invSolidAngleMeasure = imagePointToCameraDistance *
                                 imagePointToCameraDistance / cosThetaCamera;
    float revCameraPdfW = (1.0f / invSolidAngleMeasure);
    // printf("%f %f %f %f %f\n", virtualImagePlaneDistance, cosThetaCamera,
    //        imagePointToCameraDistance, invSolidAngleMeasure, revCameraPdfW);

    initPathState.origin = ray.o;
    initPathState.direction = ray.d.normalized();
    initPathState.throughput = Spectrum{1.f};
    initPathState.PathLength = 1;
    initPathState.SpecularPath = true;

    initPathState.DVC = 0.f;
    initPathState.DVCM = MIS<pow>(revCameraPdfW * camera->GetPixelCount());
    // initPathState.DVCM = MIS<pow>(  // camera->GetPixelCount()
    //     camera->A() / cRec.pdf);
    // printf("%d %f %f\n", camera->GetPixelCount(), cRec.pdf,
    // initPathState.DVCM);
  }

  Spectrum HittingLightSource(Scene* scene, Ray& ray, Intersection& isect,
                              AreaLight* light,
                              PathState& cameraPathState) const {
    float pickPdf = scene->PdfLight(light);
    float emitPdfW, directPdfA;
    Spectrum emittedRadiance =
        light->Emit(-ray.d, isect.Ng, &emitPdfW, &directPdfA);
    // printf("(%f %f %f) %f %f\n", emittedRadiance[0], emittedRadiance[1],
    //        emittedRadiance[2], emitPdfW, directPdfA);
    if (emittedRadiance.isBlack()) return {};

    // printf("%d\n", cameraPathState.PathLength);

    if (cameraPathState.PathLength == 2) {
      return emittedRadiance;
    }

    directPdfA *= pickPdf;
    emitPdfW *= pickPdf;
    float WCamera = MIS<pow>(directPdfA) * cameraPathState.DVCM +
                    MIS<pow>(emitPdfW) * cameraPathState.DVC;
    float MISWeight = 1.0f / (1.0f + WCamera);
    // printf("%f %f %f %f %f\n", directPdfA, emitPdfW, cameraPathState.DVCM,
    //        cameraPathState.DVC, WCamera);

    return MISWeight * emittedRadiance;
  }

  Spectrum ConnectToLight(const Scene* scene, Ray& pathRay,
                          const SurfaceInteraction& si, Sampler* sampler,
                          PathState& cameraPathState) const {
    // Sample light source and get radiance
    float lightPdf = 0.f;
    auto sampleLight = scene->SampleOneLight(sampler->Next1D(), &lightPdf);

    const Vector3f& pos = si.p;
    Vector3f vIn;
    VisibilityTester visibility;
    float lightPdfW;
    float cosAtLight;
    float emitPdfW;
    Spectrum radiance =
        sampleLight->Illuminate(si, sampler->Next2D(), vIn, visibility,
                                &lightPdfW, &cosAtLight, &emitPdfW);
    // printf("(%f %f %f) %f %f %f\n", radiance[0], radiance[1], radiance[2],
    //        lightPdfW, cosAtLight, emitPdfW);

    if (radiance.isBlack() || lightPdfW == 0.0f) {
      return {};
    }

    Vector3f vOut = -pathRay.d;
    Spectrum bsdfFac = si.bsdf->Evaluate(vOut, vIn);
    if (bsdfFac.isBlack()) {
      return {};
    }

    float bsdfPdfW = si.bsdf->EvaluatePdf(vOut, vIn);
    if (bsdfPdfW == 0.f) return {};
    if (sampleLight->isDelta()) bsdfPdfW = 0.f;

    float bsdfRevPdfW = si.bsdf->EvaluatePdf(vIn, vOut);

    float WLight = MIS<pow>(bsdfPdfW / (lightPdfW * lightPdf));

    // printf("%f\n", WLight);

    float cosToLight = std::abs(Math::dot(si.Ns, vIn));
    float WCamera =
        MIS<pow>(emitPdfW * cosToLight / (lightPdfW * cosAtLight)) *
        (cameraPathState.DVCM + cameraPathState.DVC * MIS<pow>(bsdfRevPdfW));

    // printf("%f\n", WCamera);

    float fMISWeight = 1.0f / (WLight + 1.0f + WCamera);
    Spectrum contribution =
        (fMISWeight * cosToLight / (lightPdfW * lightPdf)) * bsdfFac * radiance;

    if (contribution.isBlack() || !visibility.visible(*scene)) {
      return {};
    }

    // printf("%f %f %f\n", contribution[0], contribution[1], contribution[2]);

    return contribution;
  }

  Spectrum ConnectVertex(Scene* scene, SurfaceInteraction& si,
                         const Vertex& lightVertex,
                         PathState& cameraState) const {
    const Vector3f& cameraPos = si.p;

    auto dirToLight = lightVertex.si.p - cameraPos;
    float distToLightSqr = dot(dirToLight, dirToLight);
    float distToLight = dirToLight.length();

    auto vOutCam = -cameraState.direction;
    Spectrum cameraBsdfFac = si.bsdf->Evaluate(vOutCam, dirToLight);
    float cosAtCam = dot(si.Ns, dirToLight);
    auto cameraDirPdfW = si.bsdf->EvaluatePdf(vOutCam, dirToLight);
    float cameraReversePdfW = si.bsdf->EvaluatePdf(dirToLight, vOutCam);

    if (cameraBsdfFac.isBlack() || cameraDirPdfW == 0.0f ||
        cameraReversePdfW == 0.0f)
      return {};

    Vector3f dirToCamera = -dirToLight;
    Spectrum lightBsdfFac =
        lightVertex.si.bsdf->Evaluate(lightVertex.inDir, dirToCamera);
    float cosAtLight = Math::dot(lightVertex.si.Ns, dirToCamera);
    float lightDirPdfW =
        lightVertex.si.bsdf->EvaluatePdf(lightVertex.inDir, dirToCamera);
    float lightRevPdfW =
        lightVertex.si.bsdf->EvaluatePdf(dirToCamera, lightVertex.inDir);

    if (lightBsdfFac.isBlack() || lightDirPdfW == 0.0f || lightRevPdfW == 0.0f)
      return {};

    // printf("%f %f %f %f\n", cameraDirPdfW, cameraReversePdfW, lightDirPdfW,
    //        lightRevPdfW);

    float geometryTerm = cosAtLight * cosAtCam / distToLightSqr;
    if (geometryTerm < 0.0f) {
      return {};
    }

    // printf("%f\n", geometryTerm);

    float cameraDirPdfA =
        cameraDirPdfW * std::abs(cosAtLight) / (distToLight * distToLight);
    float lightDirPdfA =
        lightDirPdfW * std::abs(cosAtCam) / (distToLight * distToLight);
    float WLight =
        MIS<pow>(cameraDirPdfA) *
        (lightVertex.DVCM + lightVertex.DVC * MIS<pow>(lightRevPdfW));
    float WCamera =
        MIS<pow>(lightDirPdfA) *
        (cameraState.DVCM + cameraState.DVC * MIS<pow>(cameraReversePdfW));

    float fMISWeight = 1.0f / (WLight + 1.0f + WCamera);
    // printf("%f %f %f\n", fMISWeight, WLight, WCamera);

    Spectrum contribution =
        (fMISWeight * geometryTerm) * lightBsdfFac * cameraBsdfFac;

    Ray rayToLight = Ray(cameraPos, dirToLight, Ray::Eps(),
                         distToLight * (1.f - Ray::Eps()));
    if (contribution.isBlack() || scene->Occlude(rayToLight)) {
      // printf("%f %f %f\n", contribution[0], contribution[1],
      // contribution[2]);
      return {};
    }

    // printf("%f %f %f\n", contribution[0], contribution[1], contribution[2]);
    return contribution;
  }

  virtual Math::Spectrum Li(Core::Scene* scene, Core::Camera* camera,
                            const Math::Vector2i& raster,
                            Core::Sampler* sampler) const override {
    Vertex* lightVertices = (Vertex*)calloc(maxDepth, sizeof(Vertex));
    int numLightVertex;
    int lightPathLen = GenerateLightPath(
        scene, sampler, maxDepth + 1, lightVertices, camera, &numLightVertex);

    const float u =
        (raster.x() + sampler->Next1D()) / camera->GetFilm()->Dimension().x();
    const float v =
        (raster.y() + sampler->Next1D()) / camera->GetFilm()->Dimension().y();
    auto ray = camera->GenerateRay(u, v);
    PathState cameraPathState;
    SampleCamera(camera, ray, sampler, cameraPathState);

    Math::Spectrum L(0.f);
    while (true) {
      Ray pathRay(cameraPathState.origin, cameraPathState.direction);
      Intersection isect;
      if (!scene->Intersect(pathRay, &isect)) {
        break;
      }

      float cosIn = std::abs(dot(isect.Ng, -pathRay.d));
      cameraPathState.DVCM *= MIS<pow>(isect.t * isect.t);
      cameraPathState.DVCM /= MIS<pow>(cosIn);
      cameraPathState.DVC /= MIS<pow>(cosIn);
      // printf("%f %f\n", cameraPathState.DVCM, cameraPathState.DVC);

      if (isect.mesh->IsEmitter()) {
        cameraPathState.PathLength++;

        L += cameraPathState.throughput *
             HittingLightSource(scene, pathRay, isect,
                                isect.mesh->GetLight(isect.triId).get(),
                                cameraPathState);

        break;
      }

      if (++cameraPathState.PathLength >= maxDepth + 2) {
        break;
      }

      Triangle triangle{};
      isect.mesh->GetTriangle(isect.triId, &triangle);
      auto p = pathRay.Point(isect.t);
      SurfaceInteraction si(-pathRay.d, p, triangle, isect);
      isect.mesh->GetMaterial()->ComputeScatteringFunction(&si);

      BSDFSamplingRecord bRec(si, sampler->Next2D());
      si.bsdf->Sample(bRec);
      if (bRec.pdf <= 0.f) break;

      auto specular = bRec.type & BSDFType::BSDF_SPECULAR;
      if (!specular) {
        L += cameraPathState.throughput *
             ConnectToLight(scene, pathRay, si, sampler, cameraPathState);

        for (int i = 0; i < numLightVertex; i++) {
          const Vertex& lightVertex = lightVertices[i];

          if (lightVertex.length + cameraPathState.PathLength - 2 > maxDepth) {
            break;
          }

          L += lightVertex.throughput * cameraPathState.throughput *
               ConnectVertex(scene, si, lightVertex, cameraPathState);
        }
      }

      Spectrum f = bRec.f;
      Vector3f wi = bRec.wi;
      float scatteredPdf = bRec.pdf;

      float revPdf = si.bsdf->EvaluatePdf(wi, si.bsdf->toLocal(-pathRay.d));

      if (f.isBlack() || scatteredPdf == 0.f) break;
      // printf("(%f %f %f) %f\n", f[0], f[1], f[2], scatteredPdf);

      if (!specular && rrDepth != -1 && cameraPathState.PathLength > rrDepth) {
        float q = std::min(0.95f, cameraPathState.throughput.max());
        if (sampler->Next1D() >= q) break;
        cameraPathState.throughput /= q;
      }

      cameraPathState.origin = si.p;
      cameraPathState.direction = si.bsdf->toWorld(bRec.wi);

      float cosOut = std::abs(dot(si.Ns, si.bsdf->toWorld(bRec.wi)));
      if (!specular) {
        cameraPathState.SpecularPath &= 0;

        cameraPathState.DVCM =
            MIS<pow>(cosOut / scatteredPdf) *
            (cameraPathState.DVC * MIS<pow>(revPdf) + cameraPathState.DVCM);
        cameraPathState.DVC = MIS<pow>(1.0f / scatteredPdf);
      } else {
        cameraPathState.SpecularPath &= 1;

        cameraPathState.DVCM = 0.f;
        cameraPathState.DVC *= MIS<pow>(cosOut);
      }

      cameraPathState.throughput *= f * cosOut / scatteredPdf;
    }

    free((void*)lightVertices);

    // Li = Spectrum(0.f);

    // PathVertex* cameraVertices = new PathVertex[maxDepth + 2];
    // PathVertex* lightVertices = new PathVertex[maxDepth + 1];

    // size_t nCamera = GenerateCameraSubpath(*scene, *camera, raster, *sampler,
    //                                        maxDepth + 2, cameraVertices);
    // size_t nLight =
    //     GenerateLightSubpath(*scene, *sampler, maxDepth + 1, lightVertices);

    // Math::Spectrum L(0.f);
    // for (size_t t = 1; t <= nCamera; ++t) {
    //   for (size_t s = 0; s <= nLight; ++s) {
    //     int depth = int(t + s) - 2;
    //     if ((s == 1 && t == 1) || depth < 0 || depth > maxDepth) continue;
    //     Math::Vector2f pRaster{(float)raster.x(), (float)raster.y()};
    //     Math::Spectrum LPath = ConnectPath(*scene, *sampler, cameraVertices,
    //     t,
    //                                        lightVertices, s, &pRaster);
    //     if (t != 1) {
    //       // if (visualizeMIS) {
    //       //   pyramid.at(BufferIndex(s, t))->AddSplat(LPath, raster);
    //       // }
    //       L += LPath;
    //     } else {
    //       // if (visualizeMIS) {
    //       //   pyramid.at(BufferIndex(s, t))->AddSplat(LPath, pRaster);
    //       // }
    //       camera->GetFilm()->AddSplat(LPath, pRaster);
    //       // film->AddSplat(LPath, pRaster);
    //     }
    //   }
    // }

    // delete[] lightVertices;
    // delete[] cameraVertices;
    return L;
  }

 private:
  // int rrDepth = 5, maxDepth = 16;
  int rrDepth = 5, maxDepth = 16;

  // heuristic
  static constexpr int pow = 1;
};

}  // namespace Ajisai::Integrators

AJISAI_PLUGIN_REGISTER(BDPTIntegrator, Ajisai::Integrators::BDPTIntegrator,
                       "ajisai.integrators.Integrator/0.0.1")