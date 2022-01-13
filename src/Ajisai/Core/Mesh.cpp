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

#include <Ajisai/Core/Light.h>
#include <Ajisai/Core/Mesh.h>
#include <Ajisai/Util/ZStream.h>

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

bool Mesh::LoadSerialized(const std::filesystem::path& path, int shapeIndex) {
  using namespace Util;
  std::shared_ptr<Stream> stream = std::make_shared<FileStream>(path);

  int16_t format = 0, version = 0;
  stream->Read(&format, sizeof(int16_t));
  stream->Read(&version, sizeof(int16_t));
  std::cout << "MTS_FILEFORMAT_HEADER : " << 0x041C << " format : " << format
            << " version : " << version << std::endl;
  stream->Seek(stream->Size() - sizeof(uint32_t));

  uint32_t count = 0;
  stream->Read(&count, sizeof(uint32_t));

  stream->Seek(stream->Size() - sizeof(uint32_t) * (count - shapeIndex + 1));
  uint32_t offset = 0;
  stream->Read(&offset, sizeof(uint32_t));
  stream->Seek(offset);
  std::cout << "count : " << count << " offset : " << offset << std::endl;
  stream->Seek(stream->Tell() + 2 * sizeof(int16_t));

  std::shared_ptr<Stream> zstream = std::make_shared<ZStream>(stream.get());

  uint32_t flags = 0;
  zstream->Read(&flags, sizeof(uint32_t));

  size_t vertex_count, face_count;
  zstream->Read(&vertex_count, sizeof(size_t));
  zstream->Read(&face_count, sizeof(size_t));

  std::cout << "flags : " << flags << " vertex_count : " << vertex_count
            << " face_count : " << face_count << std::endl;

  auto has_flag = [](uint32_t flags, TriMeshFlags f) -> bool {
    return (flags & static_cast<uint32_t>(f)) != 0;
  };

  bool double_precision = has_flag(flags, TriMeshFlags::DoublePrecision);

  std::cout << "has double precision flag : " << double_precision << std::endl;

  auto read_helper = [&](Stream* stream, bool dp, std::vector<float>& dst,
                         uint32_t dim) {
    dst.resize((uint32_t)vertex_count * dim);
    if (dp) {
      std::unique_ptr<double[]> values(
          new double[(uint32_t)vertex_count * dim]);
      stream->ReadArray(values.get(), vertex_count * dim);
      for (size_t i = 0; i < vertex_count * dim; ++i) dst[i] = (float)values[i];
    } else {
      std::unique_ptr<float[]> values(new float[4 * 3]);
      // stream->ReadArray(values.get(), vertex_count * dim);
      // memcpy(dst.data(), values.get(), vertex_count * dim * sizeof(float));
    }
  };

  std::vector<float> vertices;
  vertices.resize(vertex_count * 3);
  std::unique_ptr<float[]> values(new float[vertex_count * 3]);
  zstream->ReadArray(values.get(), vertex_count * 3);
  memcpy(vertices.data(), values.get(), vertex_count * 3 * sizeof(float));

  std::vector<float> normals;
  if (has_flag(flags, TriMeshFlags::HasNormals)) {
    normals.resize(vertex_count * 3);
    std::unique_ptr<float[]> values1(new float[vertex_count * 3]);
    zstream->ReadArray(values1.get(), vertex_count * 3);
    memcpy(normals.data(), values1.get(), vertex_count * 3 * sizeof(float));
  }

  std::vector<float> coordinate;
  if (has_flag(flags, TriMeshFlags::HasTexcoords)) {
    coordinate.resize(vertex_count * 2);
    std::unique_ptr<float[]> values2(new float[vertex_count * 2]);
    zstream->ReadArray(values2.get(), vertex_count * 2);
    memcpy(coordinate.data(), values2.get(), vertex_count * 2 * sizeof(float));
  }

  std::vector<float> colors;
  if (has_flag(flags, TriMeshFlags::HasColors)) {
    colors.resize(vertex_count * 3);
    std::unique_ptr<float[]> values3(new float[vertex_count * 3]);
    zstream->ReadArray(values3.get(), vertex_count * 3);
    memcpy(colors.data(), values3.get(), vertex_count * 3 * sizeof(float));
  }

  std::vector<uint32_t> faces;
  faces.resize(face_count * 3);
  zstream->Read(faces.data(), face_count * sizeof(uint32_t) * 3);

  for (size_t i = 0; i != vertex_count; ++i) {
    Vertex vertex{};
    vertex.pos = {vertices[i * 3 + 0], vertices[i * 3 + 1],
                  vertices[i * 3 + 2]};
    if (has_flag(flags, TriMeshFlags::HasNormals)) {
      vertex.Ns = {normals[i * 3 + 0], normals[i * 3 + 1], normals[i * 3 + 2]};
    }

    if (has_flag(flags, TriMeshFlags::HasTexcoords)) {
      vertex.texCoord = {coordinate[i * 2 + 0], coordinate[i * 2 + 1]};
    }
    this->vertices.emplace_back(std::move(vertex));
  }

  if (!has_flag(flags, TriMeshFlags::HasNormals)) {
    ComputeNormal();
  }

  for (auto f : faces) {
    indices.push_back(f);
  }

  return true;
}

void Mesh::ComputeNormal() {
  for (size_t i = 0; i < vertices.size(); i += 3) {
    auto& v0 = vertices[i + 0];
    auto& v1 = vertices[i + 1];
    auto& v2 = vertices[i + 2];

    auto n = Math::cross(v1.pos - v0.pos, v2.pos - v0.pos).normalized();
    v0.Ns = n;
    v1.Ns = n;
    v2.Ns = n;
  }
}

bool Mesh::CreateRectangleMesh() {
  vertices.emplace_back(Vertex{{-1.f, -1.f, 0.f}, {0.f, 0.f, 1.f}, {}});
  vertices.emplace_back(Vertex{{1.f, -1.f, 0.f}, {0.f, 0.f, 1.f}, {}});
  vertices.emplace_back(Vertex{{1.f, 1.f, 0.f}, {0.f, 0.f, 1.f}, {}});
  vertices.emplace_back(Vertex{{-1.f, 1.f, 0.f}, {0.f, 0.f, 1.f}, {}});

  indices = {0, 1, 2, 2, 3, 0};

  return true;
}

std::vector<std::shared_ptr<AreaLight>> Mesh::GetLights() {
  if (!lights.empty()) return lights;
  for (int i = 0; i < indices.size() / 3; ++i) {
    lights.emplace_back(
        std::make_shared<AreaLight>(this, i, emitter->radiance));
    // lights.emplace_back(this, i);
  }
  return lights;
}

}  // namespace Ajisai::Core