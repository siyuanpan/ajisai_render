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
#include <ajisai/utility/hash.h>
#include <ajisai/math/bounds.h>
#include <ajisai/math/functions.h>

AJ_BEGIN

class HashGrid {
 public:
  void Reserve(int num_cells) { cell_ends_.resize(num_cells); }

  template <typename T>
  void Build(const std::vector<T>& particles, float radius) {
    radius_ = radius;
    radius_sqr_ = radius * radius;
    cell_size_ = radius_ * 2.f;
    inv_cell_size_ = 1.f / cell_size_;

    bbox_ = Bounds3f(Vector3f{std::numeric_limits<float>::max()},
                     Vector3f{std::numeric_limits<float>::lowest()});

    for (size_t i = 0; i < particles.size(); ++i) {
      const Vector3f pos = particles[i].GetPosition();
      bbox_.min() = Min(bbox_.min(), pos);
      bbox_.max() = Max(bbox_.max(), pos);
    }

    indices_.resize(particles.size());
    memset(cell_ends_.data(), 0, cell_ends_.size() * sizeof(int));

    for (size_t i = 0; i < particles.size(); ++i) {
      const auto& pos = particles[i].GetPosition();
      cell_ends_[GetCellIndex(pos)]++;
    }

    int sum = 0;
    for (size_t i = 0; i < cell_ends_.size(); ++i) {
      int temp = cell_ends_[i];
      cell_ends_[i] = sum;
      sum += temp;
    }

    for (size_t i = 0; i < particles.size(); ++i) {
      const auto& pos = particles[i].GetPosition();
      const int target_idx = cell_ends_[GetCellIndex(pos)]++;
      indices_[target_idx] = static_cast<int>(i);
    }
  }

  template <typename T, typename U>
  void Process(const std::vector<T>& particles, U& query) {
    const Vector3f query_pos = query.GetPosition();

    const Vector3f dist_min = query_pos - bbox_.min();
    const Vector3f dist_max = bbox_.max() - query_pos;

    if (dist_min.min() < 0.f || dist_max.min() < 0.f) return;

    const Vector3f cell_pt = inv_cell_size_ * dist_min;
    const Vector3f coord_f{std::floor(cell_pt.x()), std::floor(cell_pt.y()),
                           std::floor(cell_pt.z())};

    const int px = static_cast<int>(coord_f.x());
    const int py = static_cast<int>(coord_f.y());
    const int pz = static_cast<int>(coord_f.z());

    const Vector3f fract_coord = cell_pt - coord_f;

    const int pxo = px + (fract_coord.x() < 0.5f ? -1 : +1);
    const int pyo = py + (fract_coord.y() < 0.5f ? -1 : +1);
    const int pzo = pz + (fract_coord.z() < 0.5f ? -1 : +1);

    int found = 0;

    for (int j = 0; j < 8; ++j) {
      Vector2i active_range{};
      switch (j) {
        case 0:
          active_range = GetCellRange(GetCellIndex(Vector3i(px, py, pz)));
          break;
        case 1:
          active_range = GetCellRange(GetCellIndex(Vector3i(px, py, pzo)));
          break;
        case 2:
          active_range = GetCellRange(GetCellIndex(Vector3i(px, pyo, pz)));
          break;
        case 3:
          active_range = GetCellRange(GetCellIndex(Vector3i(px, pyo, pzo)));
          break;
        case 4:
          active_range = GetCellRange(GetCellIndex(Vector3i(pxo, py, pz)));
          break;
        case 5:
          active_range = GetCellRange(GetCellIndex(Vector3i(pxo, py, pzo)));
          break;
        case 6:
          active_range = GetCellRange(GetCellIndex(Vector3i(pxo, pyo, pz)));
          break;
        case 7:
          active_range = GetCellRange(GetCellIndex(Vector3i(pxo, pyo, pzo)));
          break;
      }

      for (; active_range.x() < active_range.y(); active_range.x()++) {
        const int particle_index = indices_[active_range.x()];
        const T& particle = particles[particle_index];

        const float dist_sqr =
            (query.GetPosition() - particle.GetPosition()).dot();

        if (dist_sqr <= radius_sqr_) query.Process(particle);
      }
    }
  }

 private:
  Vector2i GetCellRange(int cell_index) const {
    if (cell_index == 0) return Vector2i{0, cell_ends_[0]};
    return Vector2i{cell_ends_[cell_index - 1], cell_ends_[cell_index]};
  }

  int GetCellIndex(const Vector3i& coord) const {
    uint32_t x = static_cast<uint32_t>(coord.x());
    uint32_t y = static_cast<uint32_t>(coord.y());
    uint32_t z = static_cast<uint32_t>(coord.z());

    return int(((x * 73856093) ^ (y * 19349663) ^ (z * 83492791)) %
               uint32_t(cell_ends_.size()));
  }

  int GetCellIndex(const Vector3f& point) const {
    const Vector3f dist_min = point - bbox_.min();
    const Vector3f coord_f{std::floor(inv_cell_size_ * dist_min.x()),
                           std::floor(inv_cell_size_ * dist_min.y()),
                           std::floor(inv_cell_size_ * dist_min.z())};

    const Vector3i coord_i{static_cast<int>(coord_f.x()),
                           static_cast<int>(coord_f.y()),
                           static_cast<int>(coord_f.z())};

    return GetCellIndex(coord_i);
  }

  Bounds3f bbox_;
  std::vector<int> indices_;
  std::vector<int> cell_ends_;

  float radius_;
  float radius_sqr_;
  float cell_size_;
  float inv_cell_size_;
};

AJ_END