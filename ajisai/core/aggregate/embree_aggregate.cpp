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
#include <ajisai/core/aggregate/aggregate.h>
#include <ajisai/core/ray.h>
#include <ajisai/core/intersection.h>
#include <embree3/rtcore.h>

AJ_BEGIN

class EmbreeAggregate : public Aggregate {
 public:
  ~EmbreeAggregate() {
    if (rtc_scene_) rtcReleaseScene(rtc_scene_);

    if (rtc_device_) rtcReleaseDevice(rtc_device_);
  }

  virtual void Build(const std::vector<Rc<Primitive>>& primitives) override {
    primitives_.assign(primitives.begin(), primitives.end());

    if (!rtc_device_) rtc_device_ = rtcNewDevice(nullptr);

    if (!rtc_scene_) rtc_scene_ = rtcNewScene(rtc_device_);

    for (auto& primitive : primitives_) {
      auto mesh_view = primitive->AsGeometry()->GetMeshView();
      auto rtc_geo = rtcNewGeometry(rtc_device_, RTC_GEOMETRY_TYPE_TRIANGLE);
      rtcSetSharedGeometryBuffer(
          rtc_geo, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
          mesh_view.position_buffer, mesh_view.position_offset,
          mesh_view.position_stride, mesh_view.position_size);
      rtcSetSharedGeometryBuffer(rtc_geo, RTC_BUFFER_TYPE_INDEX, 0,
                                 RTC_FORMAT_UINT3, mesh_view.index_buffer,
                                 mesh_view.index_offset, mesh_view.index_stride,
                                 mesh_view.index_size);
      rtcCommitGeometry(rtc_geo);
      rtcAttachGeometry(rtc_scene_, rtc_geo);
      rtcReleaseGeometry(rtc_geo);
    }

    rtcCommitScene(rtc_scene_);
  }

  virtual bool Intersect(const Ray& ray,
                         PrimitiveIntersection* inct) const noexcept override {
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

    rtcIntersect1(rtc_scene_, &ctx, &rayhit);
    if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID ||
        rayhit.hit.primID == RTC_INVALID_GEOMETRY_ID) {
      return false;
    }

    inct->t = rayhit.ray.tfar;
    inct->geometry_normal =
        Vector3f{rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z}
            .normalized();
    inct->uv = Vector2f{rayhit.hit.u, rayhit.hit.v};
    primitives_[rayhit.hit.geomID]->PostIntersect(ray, inct, rayhit.hit.primID);

    // if (std::abs(inct->t - inc.t) < 1e-10)
    //   printf("(%f) (%f)\n", inct->t, inc.t);
    // if (inct->pos != inc.pos)
    //   printf("(%f %f %f) (%f %f %f)\n", inct->pos[0], inct->pos[1],
    //          inct->pos[2], inc.pos[0], inc.pos[1], inc.pos[2]);

    // if (inct->geometry_normal != inc.geometry_normal)
    //   printf("(%f %f %f) (%f %f %f)\n", inct->geometry_normal[0],
    //          inct->geometry_normal[1], inct->geometry_normal[2],
    //          inc.geometry_normal[0], inc.geometry_normal[1],
    //          inc.geometry_normal[2]);

    return true;
  }

  virtual bool Occlude(const Ray& ray) const noexcept override {
    RTCRay rtcray;
    rtcray.org_x = ray.o.x();
    rtcray.org_y = ray.o.y();
    rtcray.org_z = ray.o.z();
    rtcray.dir_x = ray.d.x();
    rtcray.dir_y = ray.d.y();
    rtcray.dir_z = ray.d.z();
    rtcray.tnear = ray.t_min;
    rtcray.tfar = ray.t_max;
    rtcray.mask = -1;
    rtcray.flags = 0;

    RTCIntersectContext ctx;
    rtcInitIntersectContext(&ctx);
    rtcOccluded1(rtc_scene_, &ctx, &rtcray);

    return rtcray.tfar < 0;
  }

 private:
  std::vector<Rc<const Primitive>> primitives_;
  RTCDevice rtc_device_;
  RTCScene rtc_scene_;
};

Rc<Aggregate> CreateEmbreeAggregate() { return RcNew<EmbreeAggregate>(); }

AJ_END