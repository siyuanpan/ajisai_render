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
#include <ajisai/core/embree.h>

AJ_BEGIN

namespace {
struct Node;
// class EmbreeAggregate;

struct Interior {
  Node *left, *right;
  Bounds3f left_bound, right_bound;
};

struct Leaf {
  size_t start, end;
};

struct Node {
  bool is_interior = true;

  union {
    Interior interior;
    Leaf leaf;
  };
};

Bounds3f Rtc2Bounds(const RTCBounds& bbox) noexcept {
  return {{bbox.lower_x, bbox.lower_y, bbox.lower_z},
          {bbox.upper_x, bbox.upper_y, bbox.upper_z}};
}

bool IntersectP(const Bounds3f& bounds, const Ray& ray, const Vector3f& inv_dir,
                const BoolVector<3>& dis_is_neg) {
  auto tMin = (Lerp(bounds.min(), bounds.max(), dis_is_neg) - ray.o) * inv_dir;
  auto tMax = (Lerp(bounds.min(), bounds.max(), ~dis_is_neg) - ray.o) * inv_dir;

  if (tMin.max() > tMax.min()) return false;
  return (tMin.max() < ray.t_max) && (tMax.min() > ray.t_min);
}

// void* CreateInterior(RTCThreadLocalAllocator alloc, unsigned int num_child,
//                      void* user_ptr) {
//   assert(num_child == 2);

//   Node* node = static_cast<Node*>(
//       rtcThreadLocalAlloc(alloc, sizeof(Node), alignof(Node)));

//   node->is_interior = true;
//   node->interior.left = nullptr;
//   node->interior.right = nullptr;
//   node->interior.left_bound = {};
//   node->interior.right_bound = {};

//   return node;
// }

// void SetInteriorChildren(void* raw_node, void** children,
//                          unsigned int num_child, void* user_ptr) {
//   assert(num_child == 2);
//   auto node = static_cast<Node*>(raw_node);
//   node->interior.left = static_cast<Node*>(children[0]);
//   node->interior.right = static_cast<Node*>(children[1]);
// }

// void SetInteriorBounds(void* raw_node, const RTCBounds** bounds,
//                        unsigned int num_child, void* user_ptr) {
//   assert(num_child == 2);
//   auto node = static_cast<Node*>(raw_node);
//   node->interior.left_bound = Rtc2Bounds(*bounds[0]);
//   node->interior.right_bound = Rtc2Bounds(*bounds[1]);
// }

// void* CreateLeaf(RTCThreadLocalAllocator alloc, const RTCBuildPrimitive*
// prims,
//                  size_t num_prims, void* user_ptr) {
//   Node* node = static_cast<Node*>(
//       rtcThreadLocalAlloc(alloc, sizeof(Node), alignof(Node)));
//   node->is_interior = false;

//   auto& params = *static_cast<EmbreeAggregate::BuildingParams*>(user_ptr);

//   const size_t start = params.output_prims.size();
//   for (size_t i = 0; i < num_prims; ++i) {
//     params.output_prims.push_back(params.all_entities[prims[i].primID].get());
//   }
//   const size_t end = params.output_prims.size();

//   node->leaf.start = start;
//   node->leaf.end = end;

//   return node;
// }

}  // namespace

class EmbreeAggregate : public Aggregate {
  using PrimitivePtr = const Primitive*;

  struct BuildingParams {
    std::vector<PrimitivePtr>& prims_ptr;
    std::vector<Rc<const Primitive>>& prims;
  };

  static void* CreateInterior(RTCThreadLocalAllocator alloc,
                              unsigned int num_child, void* user_ptr) {
    assert(num_child == 2);

    Node* node = static_cast<Node*>(
        rtcThreadLocalAlloc(alloc, sizeof(Node), alignof(Node)));

    node->is_interior = true;
    node->interior.left = nullptr;
    node->interior.right = nullptr;
    node->interior.left_bound = {};
    node->interior.right_bound = {};

    return node;
  }

  static void SetInteriorChildren(void* raw_node, void** children,
                                  unsigned int num_child, void* user_ptr) {
    assert(num_child == 2);
    auto node = static_cast<Node*>(raw_node);
    node->interior.left = static_cast<Node*>(children[0]);
    node->interior.right = static_cast<Node*>(children[1]);
  }

  static void SetInteriorBounds(void* raw_node, const RTCBounds** bounds,
                                unsigned int num_child, void* user_ptr) {
    assert(num_child == 2);
    auto node = static_cast<Node*>(raw_node);
    node->interior.left_bound = Rtc2Bounds(*bounds[0]);
    node->interior.right_bound = Rtc2Bounds(*bounds[1]);
  }

  static void* CreateLeaf(RTCThreadLocalAllocator alloc,
                          const RTCBuildPrimitive* prims, size_t num_prims,
                          void* user_ptr) {
    Node* node = static_cast<Node*>(
        rtcThreadLocalAlloc(alloc, sizeof(Node), alignof(Node)));
    node->is_interior = false;

    auto& params = *static_cast<BuildingParams*>(user_ptr);

    const size_t start = params.prims_ptr.size();
    for (size_t i = 0; i < num_prims; ++i) {
      params.prims_ptr.push_back(params.prims[prims[i].primID].get());
    }
    const size_t end = params.prims_ptr.size();

    node->leaf.start = start;
    node->leaf.end = end;

    return node;
  }

 public:
  ~EmbreeAggregate() {
    // if (rtc_scene_) rtcReleaseScene(rtc_scene_);

    // if (rtc_device_) rtcReleaseDevice(rtc_device_);

    if (bvh_) rtcReleaseBVH(bvh_);
  }

  virtual void Build(const std::vector<Rc<Primitive>>& primitives) override {
    primitives_.assign(primitives.begin(), primitives.end());

    if (bvh_) {
      rtcReleaseBVH(bvh_);
      bvh_ = nullptr;
      root_ = nullptr;
    }

    primitives_ptr_.clear();
    bvh_ = rtcNewBVH(EmbreeDevice::Inst().Get());

    std::vector<RTCBuildPrimitive> prims(primitives_.size());
    for (size_t i = 0; i < primitives_.size(); ++i) {
      const auto bbox = primitives_[i]->AABB();
      auto& p = prims[i];

      p.geomID = static_cast<unsigned int>(i);
      p.primID = static_cast<unsigned int>(i);
      p.lower_x = bbox.min().x();
      p.lower_y = bbox.min().y();
      p.lower_z = bbox.min().z();
      p.upper_x = bbox.max().x();
      p.upper_y = bbox.max().y();
      p.upper_z = bbox.max().z();
    }

    BuildingParams params_build{primitives_ptr_, primitives_};

    /* settings for BVH build */
    RTCBuildArguments arguments = rtcDefaultBuildArguments();
    arguments.byteSize = sizeof(arguments);
    arguments.buildFlags = RTC_BUILD_FLAG_NONE;
    arguments.buildQuality = RTC_BUILD_QUALITY_HIGH;
    arguments.maxBranchingFactor = 2;
    arguments.maxDepth = 128;
    arguments.sahBlockSize = 1;
    arguments.minLeafSize = 1;
    arguments.maxLeafSize = 1;
    arguments.traversalCost = 1.0f;
    arguments.intersectionCost = 1.0f;
    arguments.bvh = bvh_;
    arguments.primitives = prims.data();
    arguments.primitiveCount = prims.size();
    arguments.primitiveArrayCapacity = prims.capacity();
    arguments.createNode = CreateInterior;
    arguments.setNodeChildren = SetInteriorChildren;
    arguments.setNodeBounds = SetInteriorBounds;
    arguments.createLeaf = CreateLeaf;
    // arguments.splitPrimitive = splitPrimitive;
    // arguments.buildProgress = nullptr;
    arguments.userPtr = &params_build;

    root_ = static_cast<Node*>(rtcBuildBVH(&arguments));

    // if (!rtc_device_) rtc_device_ = rtcNewDevice(nullptr);

    // if (!rtc_scene_) rtc_scene_ = rtcNewScene(rtc_device_);

    // for (auto& primitive : primitives_) {
    //   auto mesh_view = primitive->AsGeometry()->GetMeshView();
    //   auto rtc_geo = rtcNewGeometry(rtc_device_, RTC_GEOMETRY_TYPE_TRIANGLE);
    //   rtcSetSharedGeometryBuffer(
    //       rtc_geo, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
    //       mesh_view.position_buffer, mesh_view.position_offset,
    //       mesh_view.position_stride, mesh_view.position_size);
    //   rtcSetSharedGeometryBuffer(rtc_geo, RTC_BUFFER_TYPE_INDEX, 0,
    //                              RTC_FORMAT_UINT3, mesh_view.index_buffer,
    //                              mesh_view.index_offset,
    //                              mesh_view.index_stride,
    //                              mesh_view.index_size);
    //   rtcCommitGeometry(rtc_geo);
    //   rtcAttachGeometry(rtc_scene_, rtc_geo);
    //   rtcReleaseGeometry(rtc_geo);
    // }

    // rtcCommitScene(rtc_scene_);
  }

  virtual bool Intersect(const Ray& ray,
                         PrimitiveIntersection* inct) const noexcept override {
    auto inv_dir = 1.f / ray.d;
    auto dir_is_neg = inv_dir < Vector3f{0.f};
    Ray r = ray;
    return IntersectHelper(r, inv_dir, dir_is_neg, *root_, inct);

    // RTCIntersectContext ctx;
    // rtcInitIntersectContext(&ctx);

    // RTCRayHit rayhit;
    // rayhit.ray.org_x = ray.o.x();
    // rayhit.ray.org_y = ray.o.y();
    // rayhit.ray.org_z = ray.o.z();
    // rayhit.ray.dir_x = ray.d.x();
    // rayhit.ray.dir_y = ray.d.y();
    // rayhit.ray.dir_z = ray.d.z();
    // rayhit.ray.tnear = ray.t_min;
    // rayhit.ray.tfar = ray.t_max;
    // rayhit.ray.mask = -1;
    // rayhit.ray.flags = 0;
    // rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    // rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;
    // rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    // rtcIntersect1(rtc_scene_, &ctx, &rayhit);
    // if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID ||
    //     rayhit.hit.primID == RTC_INVALID_GEOMETRY_ID) {
    //   return false;
    // }

    // inct->t = rayhit.ray.tfar;
    // inct->geometry_normal =
    //     Vector3f{rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z}
    //         .normalized();
    // inct->uv = Vector2f{rayhit.hit.u, rayhit.hit.v};
    // primitives_[rayhit.hit.geomID]->PostIntersect(ray, inct,
    // rayhit.hit.primID);

    // return true;
  }

  virtual bool Occlude(const Ray& ray) const noexcept override {
    auto inv_dir = 1.f / ray.d;
    auto dir_is_neg = inv_dir < Vector3f{0.f};
    return OccludeHelper(ray, inv_dir, dir_is_neg, *root_);

    // RTCRay rtcray;
    // rtcray.org_x = ray.o.x();
    // rtcray.org_y = ray.o.y();
    // rtcray.org_z = ray.o.z();
    // rtcray.dir_x = ray.d.x();
    // rtcray.dir_y = ray.d.y();
    // rtcray.dir_z = ray.d.z();
    // rtcray.tnear = ray.t_min;
    // rtcray.tfar = ray.t_max;
    // rtcray.mask = -1;
    // rtcray.flags = 0;

    // RTCIntersectContext ctx;
    // rtcInitIntersectContext(&ctx);
    // rtcOccluded1(rtc_scene_, &ctx, &rtcray);

    // return rtcray.tfar < 0;
  }

 private:
  bool OccludeHelper(const Ray& ray, const Vector3f& inv_dir,
                     const BoolVector<3>& dir_is_neg,
                     const Node& node) const noexcept {
    if (node.is_interior) {
      if (IntersectP(node.interior.left_bound, ray, inv_dir, dir_is_neg) &&
          OccludeHelper(ray, inv_dir, dir_is_neg, *node.interior.left))
        return true;

      if (IntersectP(node.interior.right_bound, ray, inv_dir, dir_is_neg) &&
          OccludeHelper(ray, inv_dir, dir_is_neg, *node.interior.right))
        return true;

      return false;
    }

    for (size_t i = node.leaf.start; i < node.leaf.end; ++i) {
      if (primitives_ptr_[i]->Occlude(ray)) return true;
    }

    return false;
  }

  bool IntersectHelper(Ray& ray, const Vector3f& inv_dir,
                       const BoolVector<3>& dir_is_neg, const Node& node,
                       PrimitiveIntersection* inct) const noexcept {
    if (node.is_interior) {
      const bool inct_left =
          IntersectP(node.interior.left_bound, ray, inv_dir, dir_is_neg);

      bool ret = inct_left && IntersectHelper(ray, inv_dir, dir_is_neg,
                                              *node.interior.left, inct);

      const bool inct_right =
          IntersectP(node.interior.right_bound, ray, inv_dir, dir_is_neg);

      ret |= inct_right && IntersectHelper(ray, inv_dir, dir_is_neg,
                                           *node.interior.right, inct);

      return ret;
    }

    bool ret = false;

    for (size_t i = node.leaf.start; i < node.leaf.end; ++i) {
      if (primitives_ptr_[i]->Intersect(ray, inct)) {
        ray.t_max = inct->t;
        ret = true;
      }
    }

    return ret;
  }

 private:
  std::vector<Rc<const Primitive>> primitives_;
  std::vector<PrimitivePtr> primitives_ptr_;

  // RTCDevice rtc_device_;
  // RTCScene rtc_scene_;
  RTCBVH bvh_;
  Node* root_;
};

Rc<Aggregate> CreateEmbreeAggregate() { return RcNew<EmbreeAggregate>(); }

AJ_END