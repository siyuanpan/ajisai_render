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
#include <ajisai/core/geometry/geometry.h>
#include <ajisai/core/geometry/helper.h>
#include <ajisai/core/intersection.h>
#include <ajisai/math/vector4.h>
#include <ajisai/core/embree.h>

AJ_BEGIN

std::string ErrString(RTCError err) {
  switch (err) {
    case RTC_ERROR_NONE:
      return "no error";
    case RTC_ERROR_UNKNOWN:
      return "unknown error";
    case RTC_ERROR_INVALID_ARGUMENT:
      return "invalid argument error";
    case RTC_ERROR_INVALID_OPERATION:
      return "invalid operation error";
    case RTC_ERROR_OUT_OF_MEMORY:
      return "out of memory error";
    case RTC_ERROR_UNSUPPORTED_CPU:
      return "unsupported cpu error";
    case RTC_ERROR_CANCELLED:
      return "cancelled error";
    default:
      return "unknown embree error code";
  }
}

class MeshGeo : public Geometry {
 public:
  explicit MeshGeo(std::vector<Vector4f> &&position_and_us,
                   std::vector<Vector4f> &&normal_and_vs,
                   std::vector<uint32_t> &&indices)
      : position_and_us_(std::move(position_and_us)),
        normal_and_vs_(std::move(normal_and_vs)),
        indices_(std::move(indices)) {
    aabb_ = {Vector3f{std::numeric_limits<float>::max()},
             Vector3f{std::numeric_limits<float>::lowest()}};
    surface_area_ = 0;

    for (size_t idx = 0; idx < indices_.size(); idx += 3) {
      int i = indices_[idx], j = indices_[idx + 1], k = indices_[idx + 2];
      auto e0 = position_and_us_[j].xyz() - position_and_us_[i].xyz();
      auto e1 = position_and_us_[k].xyz() - position_and_us_[i].xyz();
      surface_area_ += 0.5 * cross(e0, e1).length();
    }

    for (const auto &v : position_and_us_) {
      aabb_.min() = Min(aabb_.min(), v.xyz());
      aabb_.max() = Max(aabb_.max(), v.xyz());
    }

    RTCDevice device = EmbreeDevice::Inst().Get();
    scene_ = rtcNewScene(device);
    if (!scene_) {
      const RTCError err = rtcGetDeviceError(device);
      throw std::runtime_error(ErrString(err));
    }

    RTCGeometry geo = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    if (!geo) {
      const RTCError err = rtcGetDeviceError(device);
      throw std::runtime_error(ErrString(err));
    }

    rtcSetSharedGeometryBuffer(
        geo, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
        static_cast<const void *>(position_and_us_.data()), 0, sizeof(Vector4f),
        position_and_us_.size());

    rtcSetSharedGeometryBuffer(geo, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
                               static_cast<const void *>(indices_.data()), 0,
                               3 * sizeof(uint32_t), indices_.size() / 3);

    rtcSetGeometryBuildQuality(geo, RTC_BUILD_QUALITY_HIGH);
    rtcCommitGeometry(geo);
    rtcAttachGeometry(scene_, geo);
    rtcReleaseGeometry(geo);

    rtcSetSceneBuildQuality(scene_, RTC_BUILD_QUALITY_HIGH);
    rtcCommitScene(scene_);
  }

  virtual ~MeshGeo() {
    if (scene_) rtcReleaseScene(scene_);
  }

  virtual bool Intersect(const Ray &ray,
                         GeometryIntersection *inct) const noexcept override {
    RTCIntersectContext ctx;
    rtcInitIntersectContext(&ctx);

    RTCRayHit rayhit;
    rayhit.ray.org_x = ray.o.x();
    rayhit.ray.org_y = ray.o.y();
    rayhit.ray.org_z = ray.o.z();
    rayhit.ray.dir_x = ray.d.x();
    rayhit.ray.dir_y = ray.d.y();
    rayhit.ray.dir_z = ray.d.z();
    rayhit.ray.tnear = ray.t_min;
    rayhit.ray.tfar = ray.t_max;
    rayhit.ray.mask = -1;
    rayhit.ray.flags = 0;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(scene_, &ctx, &rayhit);
    if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) return false;

    inct->t = rayhit.ray.tfar;
    inct->geometry_normal =
        Vector3f{rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z}
            .normalized();
    inct->uv = Vector2f{rayhit.hit.u, rayhit.hit.v};
    inct->pos = ray.CalcPoint(inct->t);
    int i = indices_[rayhit.hit.primID * 3 + 0],
        j = indices_[rayhit.hit.primID * 3 + 1],
        k = indices_[rayhit.hit.primID * 3 + 2];
    inct->shading_normal =
        (1 - inct->uv.x() - inct->uv.y()) * normal_and_vs_[i].xyz() +
        inct->uv.x() * normal_and_vs_[j].xyz() +
        inct->uv.y() * normal_and_vs_[k].xyz();
    inct->uv =
        (1 - inct->uv.x() - inct->uv.y()) *
            Vector2f{position_and_us_[i].w(), normal_and_vs_[i].w()} +
        inct->uv.x() *
            Vector2f{position_and_us_[j].w(), normal_and_vs_[j].w()} +
        inct->uv.y() * Vector2f{position_and_us_[k].w(), normal_and_vs_[k].w()};
    inct->wr = -ray.d;

    return true;

    // bool hit = false;
    // for (size_t idx = 0; idx < indices_.size(); idx += 3) {
    //   int i = indices_[idx], j = indices_[idx + 1], k = indices_[idx + 2];
    //   if (IntersectWithTriangle(ray, position_and_us_[i].xyz(),
    //                             position_and_us_[j].xyz(),
    //                             position_and_us_[k].xyz(), inct)) {
    //     inct->pos = ray.CalcPoint(inct->t);
    //     inct->shading_normal =
    //         (1 - inct->uv.x() - inct->uv.y()) * normal_and_vs_[i].xyz() +
    //         inct->uv.x() * normal_and_vs_[j].xyz() +
    //         inct->uv.y() * normal_and_vs_[k].xyz();
    //     inct->uv =
    //         (1 - inct->uv.x() - inct->uv.y()) *
    //             Vector2f{position_and_us_[i].w(), normal_and_vs_[i].w()} +
    //         inct->uv.x() *
    //             Vector2f{position_and_us_[j].w(), normal_and_vs_[j].w()} +
    //         inct->uv.y() *
    //             Vector2f{position_and_us_[k].w(), normal_and_vs_[k].w()};
    //     inct->wr = -ray.d;
    //     hit = true;
    //   }
    // }

    // return hit;
  }

  virtual void PostIntersect(const Ray &ray, GeometryIntersection *inct,
                             uint32_t id) const noexcept override {
    int i = indices_[id * 3 + 0], j = indices_[id * 3 + 1],
        k = indices_[id * 3 + 2];

    inct->pos = ray.CalcPoint(inct->t);
    inct->geometry_normal =
        cross(position_and_us_[j].xyz() - position_and_us_[i].xyz(),
              position_and_us_[k].xyz() - position_and_us_[i].xyz())
            .normalized();
    inct->shading_normal =
        (1 - inct->uv.x() - inct->uv.y()) * normal_and_vs_[i].xyz() +
        inct->uv.x() * normal_and_vs_[j].xyz() +
        inct->uv.y() * normal_and_vs_[k].xyz();
    inct->uv =
        (1 - inct->uv.x() - inct->uv.y()) *
            Vector2f{position_and_us_[i].w(), normal_and_vs_[i].w()} +
        inct->uv.x() *
            Vector2f{position_and_us_[j].w(), normal_and_vs_[j].w()} +
        inct->uv.y() * Vector2f{position_and_us_[k].w(), normal_and_vs_[k].w()};
    inct->wr = -ray.d;
  }

  virtual bool Occlude(const Ray &ray) const noexcept override {
    RTCRay r{};
    r.org_x = ray.o.x();
    r.org_y = ray.o.y();
    r.org_z = ray.o.z();
    r.dir_x = ray.d.x();
    r.dir_y = ray.d.y();
    r.dir_z = ray.d.z();
    r.tnear = ray.t_min;
    r.tfar = ray.t_max;
    r.mask = -1;
    r.flags = 0;

    RTCIntersectContext ctx;
    rtcInitIntersectContext(&ctx);
    rtcOccluded1(scene_, &ctx, &r);
    return r.tfar < 0 && std::isinf(r.tfar);

    // for (size_t idx = 0; idx < indices_.size(); idx += 3) {
    //   int i = indices_[idx], j = indices_[idx + 1], k = indices_[idx + 2];
    //   if (OccludeTriangle(ray, position_and_us_[i].xyz(),
    //                       position_and_us_[j].xyz(),
    //                       position_and_us_[k].xyz())) {
    //     return true;
    //   }
    // }
    // return false;
  }

  virtual Bounds3f AABB() const noexcept override { return aabb_; }

  virtual MeshView GetMeshView() const noexcept override {
    MeshView ret{};
    ret.position_buffer = static_cast<const void *>(position_and_us_.data());
    ret.position_offset = 0;
    ret.position_stride = sizeof(Vector4f);
    ret.position_size = position_and_us_.size();
    ret.index_buffer = static_cast<const void *>(indices_.data());
    ret.index_offset = 0;
    ret.index_stride = 3 * sizeof(uint32_t);
    ret.index_size = indices_.size() / 3;

    return ret;
  }

  virtual Intersection Sample(float *pdf,
                              const Vector3f &sam) const noexcept override {
    Vector3f sam_new = sam;
    if (sam_new.x() + sam_new.y() > 1.f) {
      sam_new.x() = 1.f - sam_new.x();
      sam_new.y() = 1.f - sam_new.y();
    }
    Intersection inct;
    auto triangle_count = indices_.size() / 3;
    int idx = std::min(int(sam.z() * triangle_count), (int)triangle_count - 1);
    int i = indices_[idx], j = indices_[idx + 1], k = indices_[idx + 2];

    inct.pos = (1 - sam_new.x() - sam_new.y()) * position_and_us_[i].xyz() +
               sam_new.x() * position_and_us_[j].xyz() +
               sam_new.y() * position_and_us_[k].xyz();
    inct.uv =
        (1 - sam_new.x() - sam_new.y()) *
            Vector2f{position_and_us_[i].w(), normal_and_vs_[i].w()} +
        sam_new.x() * Vector2f{position_and_us_[j].w(), normal_and_vs_[j].w()} +
        sam_new.y() * Vector2f{position_and_us_[k].w(), normal_and_vs_[k].w()};
    inct.geometry_normal =
        cross(position_and_us_[j].xyz() - position_and_us_[i].xyz(),
              position_and_us_[k].xyz() - position_and_us_[i].xyz())
            .normalized();
    inct.shading_normal =
        (1 - sam_new.x() - sam_new.y()) * normal_and_vs_[i].xyz() +
        sam_new.x() * normal_and_vs_[j].xyz() +
        sam_new.y() * normal_and_vs_[k].xyz();

    *pdf = 1.f / surface_area_;

    return inct;
  }

  virtual Intersection Sample(const Vector3f &ref, float *pdf,
                              const Vector3f &sam) const noexcept override {
    return Sample(pdf, sam);
  }

  virtual float Pdf(const Vector3f &sample) const noexcept override {
    return 1.f / surface_area_;
  }

  virtual float Pdf(const Vector3f &ref,
                    const Vector3f &sample) const noexcept override {
    return Pdf(sample);
  }

 private:
  std::vector<Vector4f> position_and_us_;
  std::vector<Vector4f> normal_and_vs_;
  std::vector<uint32_t> indices_;
  Bounds3f aabb_;
  float surface_area_;
  RTCScene scene_ = nullptr;
};

Rc<Geometry> CreateMeshGeo(std::vector<Vector4f> &&position_and_us,
                           std::vector<Vector4f> &&normal_and_vs,
                           std::vector<uint32_t> &&indices) {
  return RcNew<MeshGeo>(std::move(position_and_us), std::move(normal_and_vs),
                        std::move(indices));
}

AJ_END