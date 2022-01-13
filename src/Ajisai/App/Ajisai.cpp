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

#include "Ajisai/Accelerator/Accelerator.h"
#include "Ajisai/Core/Camera.h"
#include "Ajisai/Core/Emitter.h"
#include "Ajisai/Core/Film.h"
#include "Ajisai/Core/Image.h"
#include "Ajisai/Core/Mesh.h"
#include "Ajisai/Core/Parallel.h"
#include "Ajisai/Core/RenderJob.h"
#include "Ajisai/Core/Scene.h"
#include "Ajisai/Integrators/Integrator.h"
#include "Ajisai/Materials/Glass.h"
#include "Ajisai/Materials/Matte.h"
#include "Ajisai/Materials/Mirror.h"
#include "Ajisai/Math/Math.h"
#include "Ajisai/PluginManager/Manager.h"

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

template <>
struct convert<Ajisai::Math::Matrix4f> {
  static Node encode(const Ajisai::Math::Matrix4f& rhs) {
    Node node;
    for (size_t i = 0; i != 15; ++i) {
      size_t col = i % 4;
      size_t row = i / 4;
      node.push_back(rhs[col][row]);
    }
    return node;
  }

  static bool decode(const Node& node, Ajisai::Math::Matrix4f& rhs) {
    if (!node.IsSequence() || node.size() != 16) return false;

    for (size_t i = 0; i != 15; ++i) {
      size_t col = i % 4;
      size_t row = i / 4;
      rhs[col][row] = node[i].as<float>();
    }

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

void load_scene_file(  // Ajisai::Integrators::RenderContext& ctx,
    Ajisai::Core::RenderJob& job, const std::filesystem::path& path) {
  using namespace Ajisai::Core;
  using namespace Ajisai::Math;
  using namespace Ajisai::Materials;
  YAML::Node config = YAML::LoadFile(path.string());

  float fov = config["camera"]["Perspective"]["fov"].as<float>();
  Vector2f res = config["camera"]["Perspective"]["res"].as<Vector2f>();

  if (config["camera"]["Perspective"]["transform"]["toWorld"].IsDefined()) {
    auto toWorld =
        config["camera"]["Perspective"]["transform"]["toWorld"].as<Matrix4f>();
    job.ctx.camera = std::make_shared<Ajisai::Core::Camera>(
        toWorld, fov * 3.14159265359f / 180.f, res);

  } else {
    Vector3f origin =
        config["camera"]["Perspective"]["transform"]["origin"].as<Vector3f>();
    Vector3f target =
        config["camera"]["Perspective"]["transform"]["target"].as<Vector3f>();

    float focus_distance =
        config["camera"]["Perspective"]["focus_distance"].as<float>();
    Vector3f up =
        config["camera"]["Perspective"]["transform"]["up"].as<Vector3f>();

    // float fov = config["camera"]["Perspective"]["fov"].as<float>();
    // Vector2f res = config["camera"]["Perspective"]["res"].as<Vector2f>();

    job.ctx.camera = std::make_shared<Ajisai::Core::Camera>(
        origin, target, focus_distance, up, fov * 3.14159265359f / 180.f, res);
  }

  // Vector3f origin =
  //     config["camera"]["Perspective"]["transform"]["origin"].as<Vector3f>();
  // Vector3f target =
  //     config["camera"]["Perspective"]["transform"]["target"].as<Vector3f>();

  // float focus_distance =
  //     config["camera"]["Perspective"]["focus_distance"].as<float>();
  // Vector3f up =
  //     config["camera"]["Perspective"]["transform"]["up"].as<Vector3f>();

  // float fov = config["camera"]["Perspective"]["fov"].as<float>();
  // Vector2f res = config["camera"]["Perspective"]["res"].as<Vector2f>();

  // job.ctx.camera = std::make_shared<Ajisai::Core::Camera>(
  //     origin, target, focus_distance, up, fov * 3.14159265359f / 180.f, res);

  for (int i = 0; i < config["shape"].size(); ++i) {
    std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
    if (config["shape"][i]["type"].as<std::string>() == "obj")
      mesh->Load(config["shape"][i]["filename"].as<std::string>());
    else if (config["shape"][i]["type"].as<std::string>() == "rectangle") {
      mesh->CreateRectangleMesh();
    } else if (config["shape"][i]["type"].as<std::string>() == "serialized") {
      mesh->LoadSerialized(config["shape"][i]["filename"].as<std::string>(),
                           config["shape"][i]["shapeIndex"].as<int>());
    }
    if (config["shape"][i]["transform"].IsDefined()) {
      if (config["shape"][i]["transform"]["toWorld"].IsDefined()) {
        mesh->Transform(
            config["shape"][i]["transform"]["toWorld"].as<Matrix4f>());
      }
      if (config["shape"][i]["transform"]["translate"].IsDefined()) {
        mesh->Translate(
            config["shape"][i]["transform"]["translate"].as<Vector3f>());
      }
    }
    if (config["shape"][i]["bsdf"]["type"].as<std::string>() == "diffuse") {
      if (config["shape"][i]["bsdf"]["srgb"].IsDefined())
        mesh->SetMaterial(
            std::make_shared<MatteMaterial>(Color3<float>::fromSrgb(
                config["shape"][i]["bsdf"]["srgb"].as<Vector3f>())));
      else if (config["shape"][i]["bsdf"]["rgb"].IsDefined())
        mesh->SetMaterial(std::make_shared<MatteMaterial>(
            Color3<float>(config["shape"][i]["bsdf"]["rgb"].as<Vector3f>())));
    } else if (config["shape"][i]["bsdf"]["type"].as<std::string>() ==
               "mirror") {
      mesh->SetMaterial(std::make_shared<MirrorMaterial>(
          Color3<float>(config["shape"][i]["bsdf"]["rgb"].as<Vector3f>())));
    } else if (config["shape"][i]["bsdf"]["type"].as<std::string>() ==
               "glass") {
      mesh->SetMaterial(std::make_shared<GlassMaterial>());
    }
    if (config["shape"][i]["emitter"].IsDefined()) {
      mesh->SetEmitter(std::make_shared<Emitter>(
          config["shape"][i]["emitter"]["radiance"].as<Vector3f>()));
    }
    job.ctx.scene->AddMesh(mesh);
  }
}

int main(int argc, char** argv) {
  using namespace Ajisai;
  using namespace Ajisai::Core;
  using namespace Ajisai::Math;
  using namespace Ajisai::Integrators;
  using namespace Ajisai::Accelerator;
  parse(argc, argv);

  std::shared_ptr<Ajisai::Core::Scene> scene =
      std::make_shared<Ajisai::Core::Scene>();

  PluginManager::Manager<Integrator> manager;

  auto integrator = manager.loadAndInstantiate("PathIntegrator");
  // auto integrator = manager.loadAndInstantiate("BDPTIntegrator");
  // manager.load("BDPTIntegrator");
  // auto integrator = manager.loadAndInstantiate("MMLTIntegrator");

  PluginManager::Manager<Accel> accel_manager;

#if AJISAI_USE_EMBREE
  auto accel = accel_manager.loadAndInstantiate("EmbreeAccel");
  // auto accel = accel_manager.loadAndInstantiate("BVHAccel");
#else
  auto accel = accel_manager.loadAndInstantiate("BVHAccel");
#endif

  // RenderContext ctx;
  // ctx.scene = scene;
  // ctx.sampler = std::make_shared<Sampler>();

  // load_scene_file(ctx, inputFile);

  // ctx.scene->SetAccel(accel);

  RenderJob job;
  job.ctx.scene = scene;
  job.ctx.sampler = std::make_shared<Sampler>();
  job.ctx.integrator = std::move(integrator);
  job.spp = 1;

  load_scene_file(job, inputFile);

  job.ctx.scene->SetAccel(accel);

  // job.ctx.integrator->Render(job.ctx.scene.get(), job.ctx.camera.get(),
  //                            job.ctx.sampler.get());

  job.Run();
  job.Join();
  auto filmUpdate = job.ctx.camera->GetFilm();

  std::cout << "outfile : " << outputFile << std::endl;
  filmUpdate->WriteImage(outputFile);

  thread_pool_finalize();

  return 0;
}
