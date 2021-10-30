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

#ifndef AJISAI_CORE_MESH_H_
#define AJISAI_CORE_MESH_H_

#include <filesystem>
#include <vector>

#include "Ajisai/Math/Math.h"

namespace Ajisai::Core {

namespace {
void split(const std::string& s, char delim, std::vector<std::string>& elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
}
}  // namespace

class Mesh {
 public:
  bool Intersect(const Ray& ray, Intersection* intersection) const {
    bool hit = false;
    for (int i = 0; i < indices.size() / 3; ++i) {
      auto v1 = vertices[indices[3 * i]];
      auto v2 = vertices[indices[3 * i + 1]];
      auto v3 = vertices[indices[3 * i + 2]];
      auto e1 = v2 - v1;
      auto e2 = v3 - v1;
      auto Ng = Math::cross(e1, e2).normalized();
      float a, f, u, v;
      auto h = Math::cross(ray.d.normalized(), e2);
      a = Math::dot(e1, h);
      if (a > -1e-6f && a < 1e-6f) continue;
      f = 1.0f / a;
      auto s = ray.o - v1;
      u = f * Math::dot(s, h);
      if (u < 0.0 || u > 1.0) continue;
      auto q = Math::cross(s, e1);
      v = f * Math::dot(ray.d.normalized(), q);
      if (v < 0.0 || u + v > 1.0) continue;
      float t = f * Math::dot(e2, q);
      if (t > ray.t_min && t < ray.t_max) {
        if (t < intersection->t) {
          intersection->Ng = Ng;
          intersection->t = t;
          hit = true;
        }
      }
      //   auto e1 = v1 - v0;
      //   auto e2 = v2 - v0;
      //   auto Ng = Math::cross(e1, e2).normalized();
      //   auto p = Math::cross(ray.d.normalized(), e2);
      //   auto det = Math::dot(e1, p);
      //   if (det > -1e-6f && a < 1e-6f) continue;
    }

    return hit;
  }
  bool Load(const std::filesystem::path& path) {
    std::ifstream t(path.string());
    std::string contents;

    t.seekg(0, std::ios::end);
    contents.reserve(t.tellg());
    t.seekg(0, std::ios::beg);

    contents.assign(std::istreambuf_iterator<char>(t),
                    std::istreambuf_iterator<char>());

    std::stringstream ss(contents);
    std::string item;
    while (std::getline(ss, item, '\n')) {
      if (item.empty() || item[0] == '#') continue;
      if (item[0] == 'v' && item[1] == ' ') {
        std::vector<std::string> tmp;
        split(item.substr(2), ' ', tmp);
        vertices.emplace_back(std::stof(tmp[0]), std::stof(tmp[1]),
                              std::stof(tmp[2]));
      } else if (item[0] == 'v' && item[1] == 'n') {
        std::vector<std::string> tmp;
        split(item.substr(3), ' ', tmp);
        normals.emplace_back(std::stof(tmp[0]), std::stof(tmp[1]),
                             std::stof(tmp[2]));
      } else if (item[0] == 'f' && item[1] == ' ') {
        std::vector<std::string> tmp;
        split(item.substr(2), ' ', tmp);
        if (tmp[0].find('/') == tmp[0].npos) {
          indices.emplace_back(std::stoul(tmp[0]) - 1);
          indices.emplace_back(std::stoul(tmp[1]) - 1);
          indices.emplace_back(std::stoul(tmp[2]) - 1);
        } else {
          indices.emplace_back(
              std::stoul(tmp[0].substr(0, tmp[0].find_first_of('/'))) - 1);
          indices.emplace_back(
              std::stoul(tmp[1].substr(0, tmp[1].find_first_of('/'))) - 1);
          indices.emplace_back(
              std::stoul(tmp[2].substr(0, tmp[2].find_first_of('/'))) - 1);
        }
      }
    }

    return true;
  }

 protected:
  std::vector<Math::Vector3f> vertices;
  std::vector<Math::Vector3f> normals;
  std::vector<std::size_t> indices;
};

}  // namespace Ajisai::Core

#endif