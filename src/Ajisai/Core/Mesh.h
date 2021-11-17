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

// #include <Ajisai/Core/Light.h>
#include <Ajisai/Math/Math.h>

#include <array>
#include <filesystem>
#include <iostream>
#include <vector>

#include "Ajisai/Core/BSDF.h"
#include "Ajisai/Core/Camera.h"
#include "Ajisai/Core/Emitter.h"
#include "Ajisai/Core/Material.h"

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

class AreaLight;

struct Vertex {
  Math::Vector3f pos, Ns;
  Math::Vector2f texCoord;
  Vertex() noexcept = default;
};

struct Triangle {
  std::array<Math::Vector3f, 3> v;
  std::array<Math::Vector2f, 3> texCoords;
  std::array<Math::Vector3f, 3> Ns;
  Math::Vector3f Ng;
  Math::Vector3f lerpNormal(const Math::Vector2f& uv) const {
    return (1 - uv.x() - uv.y()) * Ns[0] + uv.x() * Ns[1] + uv.y() * Ns[2];
  }
  Math::Vector2f lerpTexCoord(const Math::Vector2f& uv) const {
    return (1 - uv.x() - uv.y()) * texCoords[0] + uv.x() * texCoords[1] +
           uv.y() * texCoords[2];
  }
  Math::Vector3f lerpVert(const Math::Vector2f& uv) const {
    return (1 - uv.x() - uv.y()) * v[0] + uv.x() * v[1] + uv.y() * v[2];
  }
  float area() const {
    auto e1 = v[1] - v[0];
    auto e2 = v[2] - v[0];
    return Math::cross(e1, e2).length() * 0.5f;
  }

  bool Intersect(const Ray& ray, Intersection* intersection) const {
    bool hit = false;
    auto v1 = v[0];
    auto v2 = v[1];
    auto v3 = v[2];
    auto e1 = v2 - v1;
    auto e2 = v3 - v1;
    auto Ng = Math::cross(e1, e2).normalized();
    float a, f, u, v;
    auto h = Math::cross(ray.d.normalized(), e2);
    a = Math::dot(e1, h);
    if (a > -1e-6f && a < 1e-6f) return false;
    f = 1.0f / a;
    auto s = ray.o - v1;
    u = f * Math::dot(s, h);
    if (u < 0.0 || u > 1.0) return false;
    auto q = Math::cross(s, e1);
    v = f * Math::dot(ray.d.normalized(), q);
    if (v < 0.0 || u + v > 1.0) return false;
    float t = f * Math::dot(e2, q);
    if (t > ray.t_min && t < ray.t_max) {
      if (t < intersection->t) {
        intersection->Ng = Ng;
        intersection->t = t;
        intersection->uv = Math::Vector2f{u, v};
        hit = true;
      }
    }
    return hit;
  }
};

struct ScatteringEvent {
  Math::Vector3f wo;
  Math::Vector3f p;
  Math::Vector2f texCoord;
  Math::Vector3f Ng;
  Math::Vector3f Ns;
  std::shared_ptr<BSDF> bsdf;
  float rayBias = 1e-5f;
  ScatteringEvent(const Math::Vector3f& wo, const Math::Vector3f& p,
                  const Triangle& triangle, const Intersection& intersection)
      : wo(wo), p(p) {
    texCoord = triangle.lerpTexCoord(intersection.uv);
    Ns = triangle.lerpNormal(intersection.uv);
    Ng = triangle.Ng;
  }
  Ray SpawnRay(const Math::Vector3f& w) const {
    return Ray{p, w, rayBias / std::abs(Math::dot(w, Ng))};
  }
};

struct BSDFSamplingRecord {
  const Math::Vector3f wo;
  Math::Vector2f u;
  Math::Vector3f wi;
  float pdf = -1;
  Math::Spectrum f;
  inline BSDFSamplingRecord(const ScatteringEvent& event,
                            const Math::Vector2f& u)
      : wo(event.bsdf->toLocal(event.wo)), u(u) {}
};

class Mesh {
 public:
  void GetTriangle(uint32_t triId, Triangle* triangle) const {
    auto v0 = vertices[indices[triId * 3 + 0]].pos;
    auto v1 = vertices[indices[triId * 3 + 1]].pos;
    auto v2 = vertices[indices[triId * 3 + 2]].pos;
    auto e1 = v1 - v0;
    auto e2 = v2 - v0;
    auto Ng = Math::cross(e1, e2).normalized();
    triangle->v[0] = v0;
    triangle->v[1] = v1;
    triangle->v[2] = v2;
    triangle->texCoords[0] = vertices[indices[triId * 3 + 0]].texCoord;
    triangle->texCoords[1] = vertices[indices[triId * 3 + 1]].texCoord;
    triangle->texCoords[2] = vertices[indices[triId * 3 + 2]].texCoord;
    triangle->Ns[0] = vertices[indices[triId * 3 + 0]].Ns;
    triangle->Ns[1] = vertices[indices[triId * 3 + 1]].Ns;
    triangle->Ns[2] = vertices[indices[triId * 3 + 2]].Ns;
    triangle->Ng = Ng;
  }

  void computeScatteringFunctions(ScatteringEvent* event) const {
    event->bsdf = std::make_shared<BSDF>(material->color, event->Ng, event->Ns);
  }

  bool Intersect(const Ray& ray, Intersection* intersection) const {
    bool hit = false;
    for (int i = 0; i < indices.size() / 3; ++i) {
      auto v1 = vertices[indices[3 * i]].pos;
      auto v2 = vertices[indices[3 * i + 1]].pos;
      auto v3 = vertices[indices[3 * i + 2]].pos;
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
          intersection->triId = i;
          intersection->uv = Math::Vector2f{u, v};
          intersection->mesh = this;
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

  bool Load(const std::filesystem::path& path);
  //   bool Load(const std::filesystem::path& path) {
  //     tinyobj::ObjReaderConfig reader_config;
  //     reader_config.mtl_search_path = "./";

  //     tinyobj::ObjReader reader;

  //     if (!reader.ParseFromFile(path.string(), reader_config)) {
  //       if (!reader.Error().empty()) {
  //         std::cerr << "TinyObjReader: " << reader.Error();
  //       }
  //       std::exit(1);
  //     }

  //     if (!reader.Warning().empty()) {
  //       std::cout << "TinyObjReader: " << reader.Warning();
  //     }

  //     auto& attrib = reader.GetAttrib();
  //     auto& shapes = reader.GetShapes();
  //     auto& materials = reader.GetMaterials();

  //     for (size_t s = 0; s < shapes.size(); ++s) {
  //       size_t index_offset = 0;
  //       for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); ++f)
  //       {
  //         size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

  //         Vertex vertex[3];
  //         for (size_t v = 0; v < fv; ++v) {
  //           tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
  //           indices.push_back(indices.size());
  //           for (int i = 0; i < 3; ++i) {
  //             vertex[v].pos[i] = attrib.vertices[3 * idx.vertex_index + i];
  //           }
  //         }
  //         auto Ng = Math::cross(vertex[1].pos - vertex[0].pos,
  //                               vertex[2].pos - vertex[0].pos)
  //                       .normalized();
  //         for (size_t v = 0; v < fv; ++v) {
  //           tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
  //           if (idx.normal_index >= 0) {
  //             for (int i = 0; i < 3; ++i) {
  //               vertex[v].Ns[i] = attrib.normals[3 * idx.normal_index + i];
  //             }
  //           } else {
  //             vertex[v].Ns = Ng;
  //           }
  //           if (idx.texcoord_index >= 0) {
  //             for (int i = 0; i < 2; ++i) {
  //               vertex[v].texCoord[i] =
  //                   attrib.texcoords[2 * idx.texcoord_index + i];
  //             }
  //           } else {
  //             vertex[v].texCoord = Math::Vector2f(v > 0 ? 1 : 0, v & 0);
  //           }
  //         }
  //         index_offset += fv;
  //         for (auto& i : vertex) {
  //           vertices.emplace_back(i);
  //         }
  //       }
  //     }

  //     // std::ifstream t(path.string());
  //     // std::string contents;

  //     // t.seekg(0, std::ios::end);
  //     // contents.reserve(t.tellg());
  //     // t.seekg(0, std::ios::beg);

  //     // contents.assign(std::istreambuf_iterator<char>(t),
  //     //                 std::istreambuf_iterator<char>());

  void Translate(const Math::Vector3f& t) {
    for (auto& v : vertices) {
      v.pos += t;
    }
  }

  void SetMaterial(const std::shared_ptr<Material>& m) { material = m; }

  void SetEmitter(const std::shared_ptr<Emitter>& e) { emitter = e; }

  bool IsEmitter() const { return emitter.get() != nullptr; }

  Math::Spectrum Le(const Math::Vector3f& wo) const {
    // return emitter->color;
    // return {17.f, 12.f, 4.f};
    return emitter->radiance;  //{8.5f, 6.f, 2.f};
  }

  std::vector<std::shared_ptr<AreaLight>> GetLights();

  std::shared_ptr<AreaLight> GetLight(int triId) const { return lights[triId]; }

 protected:
  // std::vector<Math::Vector3f> vertices;
  // std::vector<Math::Vector3f> normals;
  std::vector<Vertex> vertices;
  std::vector<std::size_t> indices;
  std::shared_ptr<Material> material;
  std::shared_ptr<Emitter> emitter;
  std::vector<std::shared_ptr<AreaLight>> lights;
};

}  // namespace Ajisai::Core

#endif