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
#include <ajisai/factory/creator/geometry_creators.h>
#include <ajisai/factory/creator/helper.h>

#define TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_USE_MAPBOX_EARCUT
#include <tiny_obj_loader.h>

AJ_BEGIN

class QuadCreatorImpl {
 public:
  static std::string Name() { return "quad"; }

  static Rc<Geometry> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const auto transform = GetTransform(node["transform"]);

    const Vector3f a = node["a"].as<Vector3f>(Vector3f{-1.f, -1.f, 0.f});
    const Vector3f b = node["b"].as<Vector3f>(Vector3f{1.f, -1.f, 0.f});
    const Vector3f c = node["c"].as<Vector3f>(Vector3f{1.f, 1.f, 0.f});
    const Vector3f d = node["d"].as<Vector3f>(Vector3f{-1.f, 1.f, 0.f});

    const Vector2f ta = node["ta"].as<Vector2f>(Vector2f{});
    const Vector2f tb = node["tb"].as<Vector2f>(Vector2f{});
    const Vector2f tc = node["tc"].as<Vector2f>(Vector2f{});
    const Vector2f td = node["td"].as<Vector2f>(Vector2f{});

    // std::stringstream ss;
    // ss << "---\n"
    //    << "a : " << a << "\nb : " << b << "\nc : " << c << "\nd : " << d
    //    << "\nta : " << ta << "\ntb : " << tb << "\ntc : " << tc
    //    << "\ntd : " << td << "\ntransform : " << transform;

    // AJ_DEBUG("create quad with param : {}", ss.str());
    return CreateQuad(a, b, c, d, ta, tb, tc, td, transform);
  }
};

class CubeCreatorImpl {
 public:
  static std::string Name() { return "cube"; }

  static Rc<Geometry> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const auto transform = GetTransform(node["transform"]);

    return CreateCube(transform);
  }
};

class TwosidedCreatorImpl {
 public:
  static std::string Name() { return "twosided"; }

  static Rc<Geometry> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const auto internal = factory.Create<Geometry>(node["internal"]);

    return CreateTwoSided(std::move(internal));
  }
};

class MeshGeoCreatorImpl {
 public:
  static std::string Name() { return "mesh"; }

  static Rc<Geometry> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const auto transform = GetTransform(node["transform"]);

    const auto filename = node["filename"].as<std::string>();

    AJ_INFO("load {} begin", filename);

    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "./";
    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(filename, reader_config)) {
      if (!reader.Error().empty()) {
        AJ_ERROR("TinyObjReader: {}", reader.Error());
      }
      std::exit(1);
    }

    if (!reader.Warning().empty()) {
      AJ_ERROR("TinyObjReader: {}", reader.Warning());
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    // auto& materials = reader.GetMaterials();

    // indices for all shape
    // std::vector<std::vector<uint32_t>> shape_indices_arays;
    std::vector<uint32_t> shape_indices_arays;

    // position, normal, and texcoord(u, v) to be reorganized
    std::vector<Vector4f> position_and_us;
    std::vector<Vector4f> normal_and_vs;

    // index to be reorganized
    std::map<std::tuple<uint32_t, uint32_t, uint32_t>, uint32_t>
        index_from_tindex_combination;
    uint32_t index_counter = 0;

    for (size_t i_shape = 0; i_shape < shapes.size(); i_shape++) {
      // get shape
      const tinyobj::shape_t shape = shapes[i_shape];

      // construct shape indice
      // std::vector<uint32_t> shape_indices;

      for (size_t i_index = 0; i_index < shape.mesh.indices.size(); i_index++) {
        const uint32_t vertex_index =
            static_cast<uint32_t>(shape.mesh.indices[i_index].vertex_index);
        const uint32_t normal_index =
            static_cast<uint32_t>(shape.mesh.indices[i_index].normal_index);
        const uint32_t texcoord_index =
            static_cast<uint32_t>(shape.mesh.indices[i_index].texcoord_index);
        const auto tindex_combination =
            std::make_tuple(vertex_index, normal_index, texcoord_index);
        const auto& find_result =
            index_from_tindex_combination.find(tindex_combination);

        // cannot find. this combination should be a new index
        uint32_t index = 0;
        if (find_result == index_from_tindex_combination.end()) {
          // report index
          index = index_counter;

          // texcoord
          Vector2f texcoord =
              texcoord_index == -1
                  ? Vector2f(0.0f, 0.0f)
                  : Vector2f(attrib.texcoords[texcoord_index * 2 + 0],
                             attrib.texcoords[texcoord_index * 2 + 1]);

          // push new position
          Vector4f position_and_u(attrib.vertices[vertex_index * 3 + 0],
                                  attrib.vertices[vertex_index * 3 + 1],
                                  attrib.vertices[vertex_index * 3 + 2],
                                  texcoord.x());

          // dealing with normal
          Vector3f normal;
          if (attrib.normals.size() == 0) {
            size_t face_indices[3];
            face_indices[0] =
                shape.mesh.indices[(i_index / 3) * 3 + 0].vertex_index;
            face_indices[1] =
                shape.mesh.indices[(i_index / 3) * 3 + 1].vertex_index;
            face_indices[2] =
                shape.mesh.indices[(i_index / 3) * 3 + 2].vertex_index;

            Vector3f positions[3];
            for (size_t i = 0; i < 3; i++) {
              positions[i] = Vector3f(attrib.vertices[face_indices[i] * 3 + 0],
                                      attrib.vertices[face_indices[i] * 3 + 1],
                                      attrib.vertices[face_indices[i] * 3 + 2]);
            }

            // compute normal on our own
            normal =
                cross(positions[1] - positions[0], positions[2] - positions[0])
                    .normalized();
          } else {
            normal = Vector3f(attrib.normals[normal_index * 3 + 0],
                              attrib.normals[normal_index * 3 + 1],
                              attrib.normals[normal_index * 3 + 2]);
          }

          Vector4f normal_and_v(normal.x(), normal.y(), normal.z(),
                                texcoord.y());
          position_and_us.push_back(position_and_u);
          normal_and_vs.push_back(normal_and_v);

          // setup new index
          index_from_tindex_combination[tindex_combination] = index_counter++;
        } else {
          // report index
          index = find_result->second;
        }

        // push index
        shape_indices_arays.push_back(index);
      }
    }

    AJ_INFO("load {} end", filename);

    return CreateMeshGeo(std::move(position_and_us), std::move(normal_and_vs),
                         std::move(shape_indices_arays));
  }
};

template <class TGeometryCreatorImpl>
concept GeometryCreatorImpl = requires(TGeometryCreatorImpl) {
  { TGeometryCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TGeometryCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Geometry>>;
};

template <GeometryCreatorImpl TGeometryCreatorImpl>
class GeometryCreator : public TGeometryCreatorImpl {};

using QuadCreator = GeometryCreator<QuadCreatorImpl>;
using CubeCreator = GeometryCreator<CubeCreatorImpl>;
using TwosidedCreator = GeometryCreator<TwosidedCreatorImpl>;
using MeshGeoCreator = GeometryCreator<MeshGeoCreatorImpl>;

void AddGeometricFactory(Factory<Geometry>& factory) {
  factory.Add(QuadCreator::Name(), &QuadCreator::Create);
  factory.Add(CubeCreator::Name(), &CubeCreator::Create);
  factory.Add(TwosidedCreator::Name(), &TwosidedCreator::Create);
  factory.Add(MeshGeoCreator::Name(), &MeshGeoCreator::Create);
}

AJ_END