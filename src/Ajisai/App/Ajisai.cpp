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
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cxxopts.hpp>
#include <fstream>
#include <iostream>
#include <ryml/ryml.hpp>

#include "Ajisai/Core/Camera.h"
#include "Ajisai/Core/Film.h"
#include "Ajisai/Core/Image.hpp"
#include "Ajisai/Core/Integrator.h"
#include "Ajisai/Core/Mesh.h"
#include "Ajisai/Core/Parallel.hpp"
#include "Ajisai/Core/Scene.h"
#include "Ajisai/Math/Math.h"

static std::string inputFile;
static std::string outputFile;

namespace YAML {
template <>
struct convert<Ajisai::Math::Vector3f> {
  static Node encode(const Ajisai::Math::Vector3f& rhs) {
    Node node;
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());
    return node;
  }

  static bool decode(const Node& node, Ajisai::Math::Vector3f& rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x() = node[0].as<float>();
    rhs.y() = node[1].as<float>();
    rhs.z() = node[2].as<float>();
    return true;
  }
};

template <>
struct convert<Ajisai::Math::Vector2f> {
  static Node encode(const Ajisai::Math::Vector2f& rhs) {
    Node node;
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    return node;
  }

  static bool decode(const Node& node, Ajisai::Math::Vector2f& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x() = node[0].as<float>();
    rhs.y() = node[1].as<float>();
    return true;
  }
};

}  // namespace YAML

void parse(int argc, char** argv) {
  try {
    cxxopts::Options options(argv[0],
                             " - Ajisai render command line interface");
    options.positional_help("input output").show_positional_help();

    options.allow_unrecognised_options().add_options()(
        "i,input", "Input Scene File", cxxopts::value<std::string>())(
        "o,output", "Output Image File",
        cxxopts::value<std::string>()->default_value(
            "render.png"))("h,help", "show this help");

    options.parse_positional({"input", "output"});

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      std::cout << options.help() << std::endl;
      exit(0);
    }

    inputFile = result["input"].as<std::string>();
    outputFile = result["output"].as<std::string>();

  } catch (const cxxopts::OptionException& e) {
    std::cout << "error parsing options: " << e.what() << std::endl;
    exit(1);
  }
}

void split(const std::string& s, char delim, std::vector<std::string>& elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
}

std::shared_ptr<Ajisai::Core::Camera> parse(std::string& file) {
  using namespace Ajisai;

  std::ifstream t(file);
  std::string contents;

  t.seekg(0, std::ios::end);
  contents.reserve(t.tellg());
  t.seekg(0, std::ios::beg);

  contents.assign(std::istreambuf_iterator<char>(t),
                  std::istreambuf_iterator<char>());

  ryml::Tree tree = ryml::parse(ryml::to_csubstr(contents.c_str()));

  auto root = tree.rootref();
  auto origin = root["camera"]["Perspective"]["transform"]["origin"];
  auto target = root["camera"]["Perspective"]["transform"]["target"];
  auto up = root["camera"]["Perspective"]["transform"]["up"];

  std::cout << origin[0].val() << std::endl;
  Math::Vector3f ori{}, tar{}, u{};
  Math::Vector2f res{};
  float fov, focus_distance;

  origin[0] >> ori[0];
  origin[1] >> ori[1];
  origin[2] >> ori[2];

  target[0] >> tar[0];
  target[1] >> tar[1];
  target[2] >> tar[2];

  up[0] >> u[0];
  up[1] >> u[1];
  up[2] >> u[2];

  root["camera"]["Perspective"]["res"][0] >> res[0];
  root["camera"]["Perspective"]["res"][1] >> res[1];

  root["camera"]["Perspective"]["focus_istance"] >> focus_distance;
  root["camera"]["Perspective"]["fov"] >> fov;

  return std::make_shared<Ajisai::Core::Camera>(ori, tar, focus_distance, u,
                                                fov, res.aspectRatio());
}

void load_scene_file(Ajisai::Core::RenderContext& ctx,
                     const std::filesystem::path& path) {
  // YAML::Node node;        // starts out as null
  // node["key"] = "value";  // it now is a map node
  // node["seq"].push_back(
  //     "first element");  // node["seq"] automatically becomes a sequence
  // node["seq"].push_back("second element");

  // node["mirror"] = node["seq"][0];  // this creates an alias
  // node["seq"][0] = "1st element";   // this also changes node["mirror"]
  // node["mirror"] = "element #1";    // and this changes node["seq"][0] -
  // they're
  //                                   // really the "same" node

  // node["self"] = node;                 // you can even create self-aliases
  // node[node["mirror"]] = node["seq"];  // and strange loops :)
  // std::cout << node;
  using namespace Ajisai::Core;
  using namespace Ajisai::Math;

  YAML::Node config = YAML::LoadFile(path.string());

  Vector3f origin =
      config["camera"]["Perspective"]["transform"]["origin"].as<Vector3f>();
  Vector3f target =
      config["camera"]["Perspective"]["transform"]["target"].as<Vector3f>();

  float focus_distance =
      config["camera"]["Perspective"]["focus_istance"].as<float>();
  Vector3f up =
      config["camera"]["Perspective"]["transform"]["up"].as<Vector3f>();
  float fov = config["camera"]["Perspective"]["fov"].as<float>();
  Vector2f res = config["camera"]["Perspective"]["res"].as<Vector2f>();

  ctx.camera = std::make_shared<Ajisai::Core::Camera>(
      origin, target, focus_distance, up, fov, res.aspectRatio());

  for (int i = 0; i < config["shape"].size(); ++i) {
    std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
    mesh->Load(config["shape"][i]["filename"].as<std::string>());
    ctx.scene->AddMesh(mesh);
  }

  // std::cout << config["shape"]["filename"].as<std::string>() << std::endl;

  // std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
  // mesh->Load(config["shape"]["filename"].as<std::string>());
  // ctx.scene->AddMesh(mesh);
}

int main(int argc, char** argv) {
  using namespace Ajisai::Core;
  using namespace Ajisai::Math;
  parse(argc, argv);

  std::shared_ptr<Ajisai::Core::Scene> scene =
      std::make_shared<Ajisai::Core::Scene>();
  std::shared_ptr<Ajisai::Core::Integrator> integrator =
      std::make_shared<Ajisai::Core::Integrator>();

  std::shared_ptr<Camera> camera = parse(inputFile);
  std::shared_ptr<Film> film = std::make_shared<Film>(Vector2i{256, 256});
  RenderContext ctx;
  ctx.film = film;
  // ctx.camera = camera;
  ctx.scene = scene;

  load_scene_file(ctx, inputFile);

  auto tt = film->GetTile(Bounds2i{Vector2i{1, 1}, Vector2i{2, 2}});
  std::cout << "radiance : " << tt(Vector2i{1, 1}).radiance[0] << std::endl;
  // tt(Vector2i{0, 0}).radiance[0] = 10;
  tt.AddSample(Vector2i{1, 1}, Spectrum(1), 1);
  // pix.radiance[0] = 10;
  std::cout << "radiance : " << tt(Vector2i{1, 1}).radiance[0] << std::endl;

  auto task = integrator->CreateRenderTask(ctx);
  task->Start();
  task->Wait();

  auto filmUpdate = task->GetFilm();

  std::cout << "outfile : " << outputFile << std::endl;
  filmUpdate->WriteImage(outputFile);

  Ajisai::Math::Vector<int, 3> v0{1, 2, 3}, v1{v0};

  Ajisai::Math::Vector<int, 2> v2{4, 5};

  Ajisai::Math::Vector2<int> v3{v2}, v4{v3};
  std::cout << v4.x() << std::endl;

  Ajisai::Math::Vector2f v5{1.f, 2.f};
  Vector<int, 10> v6(12);
  Vector2i v7{500, 500};

  std::cout << "v7.x " << (v7 + Vector<int, 2>(1)).x() << std::endl;

  std::cout << v6[0] << std::endl;
  std::cout << v6[5] << std::endl;

  Vector2i v8 = (v7 + Vector2i(63)) / Vector2i(64);
  std::cout << v8.x() << " " << v8.y() << std::endl;

  std::mutex m;
  // parallel_for(10, [&](uint32_t i, uint32_t tid) {
  //   std::lock_guard<std::mutex> lk(m);
  //   std::cout << i << " " << tid << std::endl;
  // });

  // Array of random numbers
  const std::vector<int> numbers = {4, 2, 90, 58, 19, 59, 18, 24, 9};

  // Array for storing results
  std::vector<int> parallel_for_results(numbers.size());

  // Process of squaring the i-th number in an input array and store it
  auto square_i_th_element = [&numbers, &parallel_for_results](uint32_t i,
                                                               uint32_t tid) {
    parallel_for_results[i] = numbers[i] * numbers[i];
  };

  int count = 0;

  {
    auto start = std::chrono::high_resolution_clock::now();
    parallel_for(numbers.size(), square_i_th_element);
    // parallel_for(numbers.size(), square_i_th_element);
    // parallel_for(numbers.size(), square_i_th_element);
    // parallel_for(numbers.size(), square_i_th_element);
    // parallel_for(numbers.size(), square_i_th_element);
    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(
                     std::chrono::high_resolution_clock::now() - start)
                     .count()
              << " ms" << std::endl;
  }
  for (auto result : parallel_for_results) {
    std::cout << result << std::endl;
  }

  count = 0;
  {
    auto start = std::chrono::high_resolution_clock::now();
    parallel_for_2D(Vector2i(2), [&](Vector2i v, uint32_t) {
      std::lock_guard lk(m);
      std::cout << v.x() << " " << v.y() << std::endl;
    });
    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(
                     std::chrono::high_resolution_clock::now() - start)
                     .count()
              << " ms" << std::endl;
  }
  // std::cout << count << std::endl;

  // Bounds2i b{Vector2i{0, 0} + Vector2i{1}, Vector2i{4, 4}};
  // std::cout << b.min().x() << " " << b.min().y() << std::endl;
  // std::cout << b.max().x() << " " << b.max().y() << std::endl;

  Vector2i i{1, 2};
  Bounds2i b{i * (int)TileSize, (i + Vector2i(1)) * (int)TileSize};
  // i *= 2;
  // std::cout << i.x() << " " << i.y() << std::endl;
  // std::cout << (i * 4).x() << " " << (i * 4).y() << std::endl;
  std::cout << b.min().x() << " " << b.min().y() << std::endl;
  std::cout << b.max().x() << " " << b.max().y() << std::endl;
  std::cout << "b.size = " << b.Size().x() << " " << b.Size().y() << std::endl;

  std::cout << film->GetTile(b).bounds.min().x() << std::endl;

  Spectrum radiance = Spectrum(10);
  auto rr = radiance / 2.f;

  std::cout << "radiance / 2.f " << rr[0] << " " << rr[1] << " " << rr[2]
            << std::endl;

  Vector<float, 2> vv1{1.f, 3.f}, vv2{2.f, 2.f};
  std::cout << (vv1 - vv2)[0] << " " << (vv1 - vv2)[1] << std::endl;

  Image<int> image;
  std::cout << image(0, 0) << std::endl;
  image(0, 0) = 4;
  std::cout << image(0, 0) << std::endl;

  Vector4<int> vv4{1, 4, 2, 10};
  std::cout << vv4.xyz().x() << " " << vv4.xyz().y() << " " << vv4.xyz().z()
            << std::endl;
  vv4.xyz().x() = 1;
  vv4.xyz().y() = 2;
  vv4.xyz().z() = 3;
  std::cout << vv4.xyz().x() << " " << vv4.xyz().y() << " " << vv4.xyz().z()
            << std::endl;

  auto tmpow = pow(vv4, 2);
  for (int i = 0; i < 4; ++i) std::cout << "pow : " << tmpow[i] << std::endl;

  Vector4<int> vv3(vv4.xyz(), 12);
  std::cout << vv3.x() << " " << vv3.y() << " " << vv3.z() << " " << vv3.w()
            << std::endl;

  auto c1 = clamp(vv4, 2, 3);
  auto c2 = clamp(vv4, Vector4<int>(0, 2, 4, 1), Vector4<int>(2, 4, 5, 1));
  std::cout << c1[0] << " " << c1[1] << " " << c1[2] << " " << c1[3]
            << std::endl;
  std::cout << c2[0] << " " << c2[1] << " " << c2[2] << " " << c2[3]
            << std::endl;

  Vector<float, 2> vf2{3.f, 4.f};
  std::cout << vf2.normalized()[0] << " " << vf2.normalized()[1] << std::endl;
  std::cout << vf2.normalized().length() << " " << vf2.normalized().dot()
            << std::endl;

  Vector3f v3f = Vector3f{2.f, 1.f, 4.f}, v3f2 = Vector3f{-5.f, 3.f, 6.f};
  std::cout << v3f[0] << " " << v3f[1] << " " << v3f[2] << std::endl;
  std::cout << v3f2[0] << " " << v3f2[1] << " " << v3f2[2] << std::endl;
  Vector3f v3f3 = cross(v3f, v3f2);
  std::cout << v3f3[0] << " " << v3f3[1] << " " << v3f3[2] << std::endl;
  std::cout << cross(v3f3, v3f)[0] << " " << cross(v3f3, v3f)[1] << " "
            << cross(v3f3, v3f)[2] << std::endl;

  Vector3i aa{1, -1, 1}, bb{4, 3, 7};
  std::cout << aa[0] << " " << aa[1] << " " << aa[2] << std::endl;
  std::cout << bb[0] << " " << bb[1] << " " << bb[2] << std::endl;
  std::cout << cross(aa, bb)[0] << " " << cross(aa, bb)[1] << " "
            << cross(aa, bb)[2] << std::endl;

  return 0;
}