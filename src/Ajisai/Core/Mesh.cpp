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

#include <Ajisai/Core/Mesh.h>

#define TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_USE_MAPBOX_EARCUT
#include <tiny_obj_loader.h>

namespace Ajisai::Core {
bool Mesh::Load(const std::filesystem::path& path) {
  tinyobj::ObjReaderConfig reader_config;
  reader_config.mtl_search_path = "./";

  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(path.string(), reader_config)) {
    if (!reader.Error().empty()) {
      std::cerr << "TinyObjReader: " << reader.Error();
    }
    std::exit(1);
  }

  if (!reader.Warning().empty()) {
    std::cout << "TinyObjReader: " << reader.Warning();
  }

  auto& attrib = reader.GetAttrib();
  auto& shapes = reader.GetShapes();
  auto& materials = reader.GetMaterials();

  for (size_t s = 0; s < shapes.size(); ++s) {
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); ++f) {
      size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

      Vertex vertex[3];
      for (size_t v = 0; v < fv; ++v) {
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        indices.push_back(indices.size());
        for (int i = 0; i < 3; ++i) {
          vertex[v].pos[i] = attrib.vertices[3 * idx.vertex_index + i];
        }
      }
      auto Ng = Math::cross(vertex[1].pos - vertex[0].pos,
                            vertex[2].pos - vertex[0].pos)
                    .normalized();
      for (size_t v = 0; v < fv; ++v) {
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        if (idx.normal_index >= 0) {
          for (int i = 0; i < 3; ++i) {
            vertex[v].Ns[i] = attrib.normals[3 * idx.normal_index + i];
          }
        } else {
          vertex[v].Ns = Ng;
        }
        if (idx.texcoord_index >= 0) {
          for (int i = 0; i < 2; ++i) {
            vertex[v].texCoord[i] =
                attrib.texcoords[2 * idx.texcoord_index + i];
          }
        } else {
          vertex[v].texCoord = Math::Vector2f(v > 0 ? 1 : 0, v & 0);
        }
      }
      index_offset += fv;
      for (auto& i : vertex) {
        vertices.emplace_back(i);
      }
    }
  }

  return true;
}
}  // namespace Ajisai::Core