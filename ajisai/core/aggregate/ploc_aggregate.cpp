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
#include <cassert>
#include <variant>
#include <numeric>
#include <execution>
#include <algorithm>
#include <tbb/parallel_for.h>

AJ_BEGIN

inline constexpr size_t RoundUpLog2(size_t i, size_t p = 0) {
  return (size_t(1) << p) >= i ? p : RoundUpLog2(i, p + 1);
}

template <class Morton>
Morton MortonSplit(Morton x) {
  constexpr size_t kBitCount = sizeof(Morton) * CHAR_BIT;
  constexpr size_t kLogBits = RoundUpLog2(kBitCount);
  auto mask = Morton(-1) >> (kBitCount / 2);
  x &= mask;
  for (size_t i = kLogBits - 1, n = 1 << i; i > 0; --i, n >>= 1) {
    mask = (mask | (mask << n)) & ~(mask << (n / 2));
    x = (x | (x << n)) & mask;
  }

  return x;
}

template <class Morton>
Morton MortonEncode(Morton x, Morton y, Morton z) {
  return MortonSplit(x) | (MortonSplit(y) << 1) | (MortonSplit(z) << 2);
}

template <class Morton>
class MortonEncoder {
 public:
  static constexpr size_t kMaxGridDim = 1 << (sizeof(Morton) * CHAR_BIT / 3);

  MortonEncoder(const Bounds3f& bbox, size_t grid_dim = kMaxGridDim)
      : grid_dim_(grid_dim) {
    assert(grid_dim_ <= kMaxGridDim && "grid_dim must <= kMaxGridDim ");
    world2grid_ = (float)grid_dim_ * (1.f / bbox.size());
    grid_offset_ = -bbox.min() * world2grid_;
  }

  Morton Encode(const Vector3f& point) const {
    auto grid_pos = point * world2grid_ + grid_offset_;
    Morton x =
        std::min(Morton(grid_dim_ - 1), Morton(std::max(grid_pos[0], 0.f)));
    Morton y =
        std::min(Morton(grid_dim_ - 1), Morton(std::max(grid_pos[1], 0.f)));
    Morton z =
        std::min(Morton(grid_dim_ - 1), Morton(std::max(grid_pos[2], 0.f)));
    return MortonEncode(x, y, z);
  }

 private:
  size_t grid_dim_;
  Vector3f world2grid_, grid_offset_;
};

template <size_t BitsPerIteration>
class RadixSort {
 public:
  static constexpr size_t kBitsPerIteration = BitsPerIteration;

  template <class Key, class Value>
  static void Sort(Key* AJISAI_RESTRICT& keys, Key* AJISAI_RESTRICT& keys_copy,
                   Value* AJISAI_RESTRICT& values,
                   Value* AJISAI_RESTRICT& values_copy, size_t count,
                   size_t bit_count) {
    static constexpr size_t bucket_count = 1 << kBitsPerIteration;
    static constexpr Key mask = (Key(1) << kBitsPerIteration) - 1;

    auto buckets = std::make_unique<size_t[]>(bucket_count);
    auto buckets_accum = std::make_unique<size_t[]>(bucket_count);
    buckets_accum[0] = 0;
    for (size_t bit = 0; bit < bit_count; bit += kBitsPerIteration) {
      auto buckets_ptr = &buckets[0];
      std::fill(buckets_ptr, buckets_ptr + bucket_count, 0);

      tbb::parallel_for(tbb::blocked_range<size_t>(0, bucket_count),
                        [&](const tbb::blocked_range<size_t>& r) {
                          for (size_t i = 0; i < count; ++i) {
                            auto bucket = (keys[i] >> bit) & mask;
                            for (size_t b = r.begin(); b != r.end(); ++b) {
                              if (b == bucket) {
                                buckets_ptr[bucket]++;
                              }
                            }
                          }
                        });

      size_t sum = buckets[0];
      for (size_t i = 1; i < bucket_count; ++i) {
        buckets_accum[i] = sum;
        sum += buckets[i];
      }

      for (size_t i = 0; i < count; ++i) {
        size_t j = buckets[(keys[i] >> bit) & mask]++;
        keys_copy[j] = keys[i];
        values_copy[j] = values[i];
      }

      std::swap(keys_copy, keys);
      std::swap(values_copy, values);
    }
  }
};

// struct Leaf {
//   Bounds3f bound;
//   size_t primitive_count;
//   size_t first_primitive;
// };

// struct Interior {
//   Bounds3f bound;
// };

// using Node = std::variant<Leaf, Interior>;

struct Node {
  Bounds3f bound;
  size_t primitive_count;
  size_t first_child_or_primitive;

  bool IsLeaf() const { return primitive_count != 0; }
};

static bool IntersectP(const Bounds3f& bounds, const Ray& ray,
                       const Vector3f& inv_dir,
                       const BoolVector<3>& dir_is_neg) {
  auto t_min = (Lerp(bounds.min(), bounds.max(), dir_is_neg) - ray.o) * inv_dir;
  auto t_max =
      (Lerp(bounds.min(), bounds.max(), ~dir_is_neg) - ray.o) * inv_dir;
  if (t_min.max() > t_max.min()) return false;
  return (t_min.max() < ray.t_max) && (t_max.min() > ray.t_min);
}

/// See "Parallel Locally-Ordered Clustering for Bounding Volume Hierarchy
/// Construction", by D. Meister and J. Bittner.
class PLOCAggregate : public Aggregate {
 public:
  explicit PLOCAggregate(size_t search_radius)
      : search_radius_(search_radius) {}

  virtual void Build(const std::vector<Rc<Primitive>>& primitives) override {
    primitives_.assign(primitives.begin(), primitives.end());

    Bounds3f global_bbox{Vector3f{std::numeric_limits<float>::max()},
                         Vector3f{std::numeric_limits<float>::lowest()}};
    for (auto& primitive : primitives_) {
      global_bbox = Join(global_bbox, primitive->AABB());
    }

    auto primitive_indices = SortPrimitivesByMortonCode<uint32_t>(global_bbox);

    auto primitvie_count = primitives_.size();
    auto node_count = 2 * primitvie_count - 1;
    auto nodes = std::make_unique<Node[]>(node_count);
    auto nodes_copy = std::make_unique<Node[]>(node_count);
    auto aux_data = std::make_unique<size_t[]>(node_count * 3);

    size_t begin = node_count - primitvie_count;
    size_t end = node_count;
    size_t pre_end = end;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, primitvie_count),
                      [&](const tbb::blocked_range<size_t>& r) {
                        for (size_t i = r.begin(); i != r.end(); ++i) {
                          auto& node = nodes[begin + i];
                          //   node = Leaf{primitives_[i]->AABB(), 1, i};
                          node.bound = primitives_[i]->AABB();
                          node.primitive_count = 1;
                          node.first_child_or_primitive = i;
                        }
                      });

    while (end - begin > 1) {
      auto [next_begin, next_end] =
          Cluster(nodes.get(), nodes_copy.get(), aux_data.get(),
                  aux_data.get() + node_count, begin, end, pre_end);

      std::swap(nodes_copy, nodes);
      pre_end = end;
      begin = next_begin;
      end = next_end;
    }

    nodes_ = std::move(nodes);
    primitive_indices_ = std::move(primitive_indices);
    node_count_ = node_count;
  }

  virtual bool Intersect(const Ray& ray,
                         PrimitiveIntersection* inct) const noexcept override {
    Ray r = ray;
    auto inv_dir = 1.f / ray.d;
    auto dir_is_neg = inv_dir < Vector3f{0.f};
    return IntersectAux(r, inv_dir, dir_is_neg, inct, nodes_[0]);
  }

  virtual bool Occlude(const Ray& ray) const noexcept override {
    Ray r = ray;
    auto inv_dir = 1.f / ray.d;
    auto dir_is_neg = inv_dir < Vector3f{0.f};
    return OccludeAux(r, inv_dir, dir_is_neg, nodes_[0]);
  }

 private:
  bool OccludeAux(Ray& ray, const Vector3f& inv_dir,
                  const BoolVector<3>& dir_is_neg,

                  const Node& node) const noexcept {
    if (!IntersectP(node.bound, ray, inv_dir, dir_is_neg)) return false;

    if (node.IsLeaf()) {
      if (primitives_[node.first_child_or_primitive]->Occlude(ray)) {
        return true;
      }
      return false;
    }

    return OccludeAux(ray, inv_dir, dir_is_neg,
                      nodes_[node.first_child_or_primitive + 0]) ||
           OccludeAux(ray, inv_dir, dir_is_neg,
                      nodes_[node.first_child_or_primitive + 1]);
  }

  bool IntersectAux(Ray& ray, const Vector3f& inv_dir,
                    const BoolVector<3>& dir_is_neg,
                    PrimitiveIntersection* inct,
                    const Node& node) const noexcept {
    if (!IntersectP(node.bound, ray, inv_dir, dir_is_neg)) return false;

    if (node.IsLeaf()) {
      bool ret = false;
      if (primitives_[node.first_child_or_primitive]->Intersect(ray, inct)) {
        ray.t_max = inct->t;
        ret = true;
      }
      return ret;
    }

    const bool left = IntersectAux(ray, inv_dir, dir_is_neg, inct,
                                   nodes_[node.first_child_or_primitive + 0]);
    const bool right = IntersectAux(ray, inv_dir, dir_is_neg, inct,
                                    nodes_[node.first_child_or_primitive + 1]);
    return left || right;
  }

  std::pair<size_t, size_t> SearchRange(size_t i, size_t begin,
                                        size_t end) const {
    return std::make_pair(
        i > begin + search_radius_ ? i - search_radius_ : begin,
        std::min(i + search_radius_ + 1, end));
  }

  std::pair<size_t, size_t> Cluster(const Node* AJISAI_RESTRICT input,
                                    Node* AJISAI_RESTRICT output,
                                    size_t* AJISAI_RESTRICT neighbor,
                                    size_t* AJISAI_RESTRICT merged_index,
                                    size_t begin, size_t end, size_t pre_end) {
    size_t next_begin = 0;
    size_t next_end = 0;

    tbb::parallel_for(
        tbb::blocked_range<size_t>(begin, end),
        [&](const tbb::blocked_range<size_t>& r) {
          auto distances =
              std::make_unique<float[]>(search_radius_ * (search_radius_ + 1));
          auto distance_matrix = std::make_unique<float*[]>(search_radius_ + 1);
          for (size_t i = 0; i <= search_radius_; ++i)
            distance_matrix[i] = &distances[i * search_radius_];
          size_t chunk_begin = r.begin();
          size_t chunk_end = r.end();
          for (size_t i = SearchRange(chunk_begin, begin, end).first;
               i < chunk_begin; ++i) {
            auto search_end = SearchRange(i, begin, end).second;
            for (size_t j = i + 1; j < search_end; ++j) {
              distance_matrix[chunk_begin - i][j - i - 1] =
                  Join(input[i].bound, input[j].bound).area();
            }
          }

          for (size_t i = chunk_begin; i < chunk_end; ++i) {
            auto [search_begin, search_end] = SearchRange(i, begin, end);
            float best_distance = std::numeric_limits<float>::max();
            size_t best_neighbor = -1;

            for (size_t j = search_begin; j < i; ++j) {
              auto distance = distance_matrix[i - j][i - j - 1];
              if (distance < best_distance) {
                best_distance = distance;
                best_neighbor = j;
              }
            }

            for (size_t j = i + 1; j < search_end; ++j) {
              auto distance = Join(input[i].bound, input[j].bound).area();
              distance_matrix[0][j - i - 1] = distance;
              if (distance < best_distance) {
                best_distance = distance;
                best_neighbor = j;
              }
            }

            assert(best_neighbor != size_t(-1) &&
                   "best neighbor must not = -1");
            neighbor[i] = best_neighbor;

            auto last = distance_matrix[search_radius_];
            std::move_backward(distance_matrix.get(),
                               distance_matrix.get() + search_radius_,
                               distance_matrix.get() + search_radius_ + 1);
            distance_matrix[0] = last;
          }
        });

    tbb::parallel_for(tbb::blocked_range<size_t>(begin, end),
                      [&](tbb::blocked_range<size_t>& r) {
                        for (size_t i = r.begin(); i != r.end(); ++i) {
                          auto j = neighbor[i];
                          bool is_mergeable = neighbor[j] == i;
                          merged_index[i] = i < j && is_mergeable ? 1 : 0;
                        }
                      });

    std::inclusive_scan(std::execution::par, merged_index + begin,
                        merged_index + end, merged_index + begin);
    size_t merged_count = merged_index[end - 1];
    size_t unmerged_count = end - begin - merged_count;
    size_t children_count = merged_count * 2;
    size_t children_begin = end - children_count;
    size_t unmerged_begin = end - (children_count + unmerged_count);

    next_begin = unmerged_begin;
    next_end = children_begin;

    tbb::parallel_for(
        tbb::blocked_range<size_t>(begin, end),
        [&](tbb::blocked_range<size_t>& r) {
          for (size_t i = r.begin(); i != r.end(); ++i) {
            auto j = neighbor[i];
            if (neighbor[j] == i) {
              if (i < j) {
                auto& unmerged_node =
                    output[unmerged_begin + j - begin - merged_index[j]];
                auto first_child = children_begin + (merged_index[i] - 1) * 2;
                unmerged_node.bound = Join(input[j].bound, input[i].bound);
                unmerged_node.primitive_count = 0;
                unmerged_node.first_child_or_primitive = first_child;
                output[first_child + 0] = input[i];
                output[first_child + 1] = input[j];
              }
            } else {
              output[unmerged_begin + i - begin - merged_index[i]] = input[i];
            }
          }
        });

    tbb::parallel_for(tbb::blocked_range<size_t>(end, pre_end),
                      [&](tbb::blocked_range<size_t>& r) {
                        for (size_t i = r.begin(); i != r.end(); ++i) {
                          output[i] = input[i];
                        }
                      });

    return std::make_pair(next_begin, next_end);
  }

  template <class Morton>
  std::unique_ptr<size_t[]> SortPrimitivesByMortonCode(
      const Bounds3f& global_bbox) {
    static constexpr size_t kBitCount = (sizeof(Morton) * CHAR_BIT) / 3;
    auto primitive_count = primitives_.size();

    auto morton_codes = std::make_unique<Morton[]>(primitive_count);
    auto morton_codes_copy = std::make_unique<Morton[]>(primitive_count);
    auto primitive_indices = std::make_unique<size_t[]>(primitive_count);
    auto primitive_indices_copy = std::make_unique<size_t[]>(primitive_count);

    Morton* sorted_morton_codes = morton_codes.get();
    size_t* sorted_primitive_indices = primitive_indices.get();
    Morton* unsorted_morton_codes = morton_codes_copy.get();
    size_t* unsorted_primitive_indices = primitive_indices_copy.get();

    MortonEncoder<Morton> encoder(global_bbox);

    tbb::parallel_for(tbb::blocked_range<size_t>(0, primitive_count),
                      [&](const tbb::blocked_range<size_t>& r) {
                        for (size_t i = r.begin(); i != r.end(); ++i) {
                          morton_codes[i] =
                              encoder.Encode(primitives_[i]->AABB().center());
                          primitive_indices[i] = i;
                        }
                      });

    RadixSort<10>::Sort(sorted_morton_codes, unsorted_morton_codes,
                        sorted_primitive_indices, unsorted_primitive_indices,
                        primitive_count, kBitCount * 3);

    assert(std::is_sorted(morton_codes.get(),
                          morton_codes.get() + primitive_count) &&
           "morton codes must be sorted!");
    return std::move(primitive_indices);
  }

 private:
  std::vector<Rc<const Primitive>> primitives_;
  size_t search_radius_ = 14;
  std::unique_ptr<Node[]> nodes_;
  std::unique_ptr<size_t[]> primitive_indices_;
  size_t node_count_;
};

Rc<Aggregate> CreatePLOCAggregate(size_t search_radius) {
  return RcNew<PLOCAggregate>(search_radius);
}

AJ_END