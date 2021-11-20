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

#include <Ajisai/Math/Math.h>

namespace Ajisai::Accelerator {

enum class SplitMethod { SAH, HLBVH, Middle, EqualCounts };

struct Primitive {
  const Core::Mesh* mesh;
  Core::Triangle triangle;
  Math::Bounds3f bounds;
  Math::Vector3f centroid;
  int triId;
};

struct BVHNode {
  Math::Bounds3f bounds;
  int left = -1;
  int right = -1;
  uint32_t first;
  uint32_t count = (uint32_t)0;
};

static bool IntersectP(const Math::Bounds3f& bounds, const Core::Ray& ray,
                       const Math::Vector3f& invDir,
                       const Math::BoolVector<3>& dirIsNeg) {
  auto tMin =
      (Math::lerp(bounds.min(), bounds.max(), dirIsNeg) - ray.o) * invDir;
  auto tMax =
      (Math::lerp(bounds.min(), bounds.max(), ~dirIsNeg) - ray.o) * invDir;
  // std::cout << tMin.max() << " " << tMax.min() << std::endl;
  if (tMin.max() > tMax.min()) return false;
  return (tMin.max() < ray.t_max) && (tMax.min() > ray.t_min);
}

class BVHAccel final : public Accel {
 public:
  BVHAccel(PluginManager::AbstractManager& manager, const std::string& plugin)
      : Accel{manager, plugin} {}

  virtual void Build(const Core::Scene* scene) override {
    for (auto& mesh : scene->GetMeshes()) {
      for (std::size_t i = 0; i != mesh->GetTriSize(); ++i) {
        Core::Triangle triangle;
        mesh->GetTriangle(i, &triangle);

        Primitive primitive{};
        primitive.mesh = mesh.get();
        primitive.bounds = triangle.Bounds();
        primitive.centroid = triangle.Centroid();
        primitive.triangle = std::move(triangle);
        primitive.triId = i;

        primitives.emplace_back(primitive);
      }
    }

    recursiveBuild(0, primitives.size(), 0);
  }

  virtual bool Intersect(const Core::Ray& ray,
                         Core::Intersection* intersection) const {
    bool hit = false;
    auto invDir = 1.f / ray.d;
    auto dirIsNeg = invDir < Math::Vector3f(0);
    int sp = 0;
    const int maxDepth = 64;
    int stack[maxDepth];
    stack[sp++] = 0;

    while (true) {
      const auto& node = nodes[stack[--sp]];

      if (IntersectP(node.bounds, ray, invDir, dirIsNeg)) {
        if (node.count > 0) {
          for (std::size_t i = 0; i != node.count; ++i) {
            if (primitives[node.first + i].triangle.Intersect(ray,
                                                              intersection)) {
              hit = true;
              intersection->mesh = primitives[node.first + i].mesh;
              intersection->triId = primitives[node.first + i].triId;
              // std::cout << "hit\n";
            }
          }
          // std::exit(1);
          // if (sp == 0) break;
        } else {
          stack[sp++] = node.left;
          stack[sp++] = node.right;
        }
      }
      // else {
      //   // if (sp == 0) break;
      // }
      if (sp == 0) break;
      // break;
    }

    return hit;
  }

  int recursiveBuild(std::size_t start, std::size_t end, std::size_t depth) {
    using Math::Bounds3f;
    using Math::Vector3f;

    Bounds3f bounds(Vector3f{std::numeric_limits<float>::max()},
                    Vector3f{std::numeric_limits<float>::lowest()});

    for (std::size_t i = start; i != end; ++i) {
      bounds = Math::join(
          bounds, primitives[i].bounds);  // primitives[i].triangle.Bounds());
    }

    if (start == end) {
      std::exit(1);
    } else if (end - start == 1) {
      BVHNode node;
      node.bounds = bounds;
      node.first = start;
      // assert(end - start != 0);
      node.count = end - start;
      nodes.push_back(node);
      return nodes.size() - 1;
    } else {
      Bounds3f centroidBounds(Vector3f{std::numeric_limits<float>::max()},
                              Vector3f{std::numeric_limits<float>::lowest()});
      for (std::size_t i = start; i != end; ++i) {
        centroidBounds.min() =
            Math::min(centroidBounds.min(), primitives[i].centroid);
        centroidBounds.max() =
            Math::max(centroidBounds.max(), primitives[i].centroid);
      }

      auto extent = centroidBounds.size();
      std::size_t axis;
      if (extent.x() > extent.y()) {
        if (extent.x() > extent.z()) {
          axis = 0;
        } else {
          axis = 2;
        }
      } else {
        if (extent.y() > extent.z()) {
          axis = 1;
        } else {
          axis = 2;
        }
      }

      int mid = start + (end - start) / 2;
      if (centroidBounds.max()[axis] == centroidBounds.min()[axis]) {
        BVHNode node;
        node.bounds = bounds;
        node.first = start;
        // assert(end - start != 0);
        node.count = end - start;
        nodes.push_back(node);
        return nodes.size() - 1;

      } else {
        auto splitMethod = SplitMethod::SAH;
        switch (splitMethod) {
          case SplitMethod::Middle: {
            // TODO
            std::exit(1);
          }
          case SplitMethod::EqualCounts: {
            // TODO
            std::exit(1);
          }
          case SplitMethod::SAH:
          default: {
            if (end - start <= 4) {
              std::nth_element(&primitives[start], &primitives[mid],
                               &primitives[end - 1] + 1,

                               [axis](const Primitive& a, const Primitive& b) {
                                 return a.centroid[axis] < b.centroid[axis];
                               });
            } else {
              constexpr int nBuckets = 12;
              struct BucketInfo {
                int count;
                Bounds3f bounds;
                BucketInfo()
                    : count(0),
                      bounds(Vector3f{std::numeric_limits<float>::max()},
                             Vector3f{std::numeric_limits<float>::lowest()}) {}
              };
              BucketInfo buckets[nBuckets];
              for (std::size_t i = start; i < end; ++i) {
                auto o =
                    primitives[i].centroid[axis] - centroidBounds.min()[axis];
                o /= centroidBounds.max()[axis] - centroidBounds.min()[axis];
                int b = nBuckets * o;
                if (b == nBuckets) b = nBuckets - 1;
                buckets[b].count++;
                buckets[b].bounds =
                    Math::join(buckets[b].bounds, primitives[i].bounds);
              }

              float cost[nBuckets - 1];
              for (int i = 0; i < nBuckets - 1; ++i) {
                Bounds3f b0{Vector3f{std::numeric_limits<float>::max()},
                            Vector3f{std::numeric_limits<float>::lowest()}},
                    b1{Vector3f{std::numeric_limits<float>::max()},
                       Vector3f{std::numeric_limits<float>::lowest()}};
                int count0 = 0, count1 = 0;
                for (int j = 0; j <= i; ++j) {
                  b0 = Math::join(b0, buckets[j].bounds);
                  count0 += buckets[j].count;
                }
                for (int j = i + 1; j < nBuckets; ++j) {
                  b1 = Math::join(b1, buckets[j].bounds);
                  count1 += buckets[j].count;
                }

                float cost0 = count0 == 0 ? 0.f : (float)count0 * b0.area();
                float cost1 = count1 == 0 ? 0.f : (float)count1 * b1.area();
                cost[i] = .125f + (cost0 + cost1) / bounds.area();
              }
              float minCost = cost[0];
              int minCostSplitBucket = 0;
              for (int i = 1; i < nBuckets - 1; ++i) {
                if (cost[i] < minCost) {
                  minCost = cost[i];
                  minCostSplitBucket = i;
                }
              }
              auto pmid = std::partition(
                  &primitives[start], &primitives[end - 1] + 1,
                  [=](const Primitive& pi) {
                    auto o = pi.centroid[axis] - centroidBounds.min()[axis];
                    o /=
                        centroidBounds.max()[axis] - centroidBounds.min()[axis];
                    int b = nBuckets * o;
                    if (b == nBuckets) b = nBuckets - 1;
                    return b <= minCostSplitBucket;
                  });
              mid = pmid - &primitives[0];
            }
            break;
          }
        }
        auto idx = nodes.size();
        BVHNode node;
        node.bounds = bounds;
        node.count = 0;
        nodes.push_back(node);
        nodes[idx].left = recursiveBuild(start, mid, depth + 1);
        nodes[idx].right = recursiveBuild(mid, end, depth + 1);

        return idx;
      }
    }
  }

 private:
  std::vector<Primitive> primitives;
  std::vector<BVHNode> nodes;
};

}  // namespace Ajisai::Accelerator

AJISAI_PLUGIN_REGISTER(BVHAccel, Ajisai::Accelerator::BVHAccel,
                       "ajisai.accelerator.Accelerator/0.0.1")