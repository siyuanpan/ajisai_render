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
#pragma once
#include <ajisai/ajisai.h>
#include <ajisai/utility/mem_arena.h>
#include <ajisai/utility/hash.h>

AJ_BEGIN

namespace sppm {

constexpr int SPECTRUM_COMPONENT_COUNT = 3;

template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
void AtomicAdd(std::atomic<T> &original, T add_val) {
  T cur_val = original.load(std::memory_order_consume);
  T new_val = cur_val + add_val;
  while (!original.compare_exchange_weak(
      cur_val, new_val, std::memory_order_release, std::memory_order_consume)) {
    new_val = cur_val + add_val;
  }
}

struct VisiblePoint {
  Vector3f pos;
  Vector3f wr;
  Spectrum throughput;
  const BSDF *bsdf = nullptr;
  bool IsValid() const noexcept { return bsdf != nullptr; }

  // ~VisiblePoint() {
  //   if (bsdf) delete bsdf;
  // }
};

struct Pixel {
  Pixel() = default;

  VisiblePoint vp;

  float radius = 0.1f;
  std::atomic<float> phi[SPECTRUM_COMPONENT_COUNT] = {0};
  std::atomic<int> M = 0;

  float N = 0;
  Spectrum tau;

  Spectrum direct_illum{};
};

class VisiblePointSearcher {
 public:
  explicit VisiblePointSearcher(const Bounds3f &bound, float grid_sidelen,
                                size_t entry_count)
      : bound_{bound}, grid_sidelen_{grid_sidelen}, entry_count_{entry_count} {
    node_entry_ = BoxNew<std::atomic<VPNode *>[]>(entry_count_);

    Clear();
  }

  void Clear() {
    for (size_t i = 0; i < entry_count_; ++i) node_entry_[i] = nullptr;
  }

  void AddVp(Pixel &pixel, MemoryArena &arena) {
    const auto min_grid = ToGrid(pixel.vp.pos - Vector3f{pixel.radius});
    const auto max_grid = ToGrid(pixel.vp.pos + Vector3f{pixel.radius});

    for (int z = min_grid.z(); z < max_grid.z(); ++z)
      for (int y = min_grid.y(); y < max_grid.y(); ++y)
        for (int x = min_grid.x(); x < max_grid.x(); ++x) {
          const size_t entry_idx = ToEntry(Vector3i{x, y, z});
          auto &entry = node_entry_[entry_idx];

          VPNode *node = arena.Create<VPNode>();
          node->pixel = &pixel;
          node->next = entry;

          while (!entry.compare_exchange_weak(node->next, node))
            ;
        }
  }

  void AddPhoton(const Vector3f &photon_pos, const Spectrum &phi,
                 const Vector3f &wr) {
    const size_t entry_idx = ToEntry(ToGrid(photon_pos));
    for (VPNode *node = node_entry_[entry_idx]; node; node = node->next) {
      auto &pixel = *node->pixel;
      if ((pixel.vp.pos - photon_pos).dot() > pixel.radius * pixel.radius) {
        continue;
      }

      const Spectrum delta_phi =
          phi * pixel.vp.bsdf->EvalAll(wr, pixel.vp.wr, TransMode::Radiance);

      if (!delta_phi.IsFinite()) continue;

      for (size_t i = 0; i < SPECTRUM_COMPONENT_COUNT; ++i)
        AtomicAdd(pixel.phi[i], delta_phi[i]);
      ++pixel.M;
    }
  }

 private:
  Vector3i ToGrid(const Vector3f &pos) const noexcept {
    const auto hd = pos - bound_.min();
    return {
        static_cast<int>(std::max(hd.x(), 0.f) / grid_sidelen_),
        static_cast<int>(std::max(hd.y(), 0.f) / grid_sidelen_),
        static_cast<int>(std::max(hd.z(), 0.f) / grid_sidelen_),
    };
  }

  size_t ToEntry(const Vector3i &grid) const noexcept {
    const size_t low3_bit =
        ((grid.x() & 1) << 0) | ((grid.y() & 1) << 1) | ((grid.z() & 2) << 0);
    const size_t hash_v = Hash(grid.x(), grid.y(), grid.z());
    return ((hash_v << 3) % entry_count_) | low3_bit;
  }

 private:
  struct VPNode {
    Pixel *pixel = nullptr;
    VPNode *next = nullptr;
  };

  Bounds3f bound_;
  float grid_sidelen_;
  size_t entry_count_;

  Box<std::atomic<VPNode *>[]> node_entry_;
};

}  // namespace sppm

AJ_END