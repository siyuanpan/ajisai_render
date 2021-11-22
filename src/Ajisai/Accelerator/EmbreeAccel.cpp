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

#include <Ajisai/Accelerator/Accelerator.h>
#include <Ajisai/Core/Scene.h>

#include <embree3/rtcore.h>

namespace Ajisai::Accelerator {

class EmbreeAccel final : public Accel {
 public:
  EmbreeAccel(PluginManager::AbstractManager& manager,
              const std::string& plugin)
      : Accel{manager, plugin} {}

  virtual ~EmbreeAccel() {
    if (rtcScene) {
      rtcReleaseScene(rtcScene);
    }

    if (rtcDevice) {
      rtcReleaseDevice(rtcDevice);
    }
  }

  virtual void Build(const Core::Scene* scene) {
    if (!rtcDevice) rtcDevice = rtcNewDevice(nullptr);

    if (!rtcScene) rtcScene = rtcNewScene(rtcDevice);

    for (auto& mesh : scene->GetMeshes()) {
      auto rtcGeo = rtcNewGeometry(rtcDevice, RTC_GEOMETRY_TYPE_TRIANGLE);
      rtcSetSharedGeometryBuffer(rtcGeo, RTC_BUFFER_TYPE_VERTEX, 0,
                                 RTC_FORMAT_FLOAT3, mesh->GetVertexBuffer(), 0,
                                 sizeof(Core::Vertex), mesh->GetTriSize() * 3);
      rtcSetSharedGeometryBuffer(rtcGeo, RTC_BUFFER_TYPE_INDEX, 0,
                                 RTC_FORMAT_UINT3, mesh->GetIndexBuffer(), 0,
                                 3 * sizeof(uint32_t), mesh->GetTriSize());
      sizeof(std::size_t);
      sizeof(unsigned int);
      sizeof(uint64_t);
      rtcCommitGeometry(rtcGeo);
      rtcAttachGeometry(rtcScene, rtcGeo);
      rtcReleaseGeometry(rtcGeo);
    }

    rtcCommitScene(rtcScene);
  }

  virtual bool Intersect(const Core::Ray& ray,
                         Core::Intersection* intersection) const {
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

    rtcIntersect1(rtcScene, &ctx, &rayhit);
    if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID ||
        rayhit.hit.primID == RTC_INVALID_GEOMETRY_ID)
      return false;

    // if (rayhit.hit.geomID == 7) std::cout << rayhit.hit.geomID << std::endl;

    intersection->triId = rayhit.hit.primID;
    intersection->meshId = rayhit.hit.geomID;
    intersection->Ng =
        Math::Vector3f(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z)
            .normalized();
    intersection->uv = Math::Vector2f(rayhit.hit.u, rayhit.hit.v);
    intersection->t = rayhit.ray.tfar;

    return true;
  }

  virtual bool Occlude(const Core::Ray& ray) const {
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
    rtcOccluded1(rtcScene, &ctx, &rtcray);

    return rtcray.tfar < 0;
  }

 private:
  RTCDevice rtcDevice;
  RTCScene rtcScene;
};

}  // namespace Ajisai::Accelerator

AJISAI_PLUGIN_REGISTER(EmbreeAccel, Ajisai::Accelerator::EmbreeAccel,
                       "ajisai.accelerator.Accelerator/0.0.1")