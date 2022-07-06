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
#include <ajisai/utility/log.h>
#include <ajisai/factory/factory.h>
#include <ajisai/core/sampler/sampler.h>

#include <cxxopts.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>

struct Param {
  std::filesystem::path scene_name;
};

Param parse_opts(int argc, char* argv[]) {
  try {
    cxxopts::Options options(argv[0],
                             " - Ajisai render command line interface");

    options.allow_unrecognised_options().add_options()(
        "s,scene", "Input Scene File", cxxopts::value<std::string>())(
        "h,help", "Help Information");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      AJ_INFO(options.help());
      exit(0);
    }

    Param ret;

    if (result.count("scene") != 0) {
      ret.scene_name = result["scene"].as<std::string>();
    } else {
      AJ_ERROR("scene file is unspecified");
      exit(0);
    }

    return ret;

  } catch (const cxxopts::OptionException& e) {
    AJ_ERROR("error parsing options: {}", e.what());
    exit(1);
  }
}

void run(int argc, char* argv[]) {
  using namespace aj;
  auto params = parse_opts(argc, argv);

  AJ_INFO(">>> Loading file : {} <<<", params.scene_name.string());
  auto config = YAML::LoadFile(params.scene_name.string());

  AJ_INFO(">>> loading sucessed <<<");

  const auto& scene_config = config["scene"];
  const auto& camera_config = config["camera"];
  const auto& renderer_config = config["renderer"];
  const auto& post_processor_config = config["post_processors"];

  AJ_INFO(">>> Ceate Scene <<<");
  CreateFactory factory;
  auto scene = factory.Create<Scene>(scene_config);

  AJ_INFO(">>> Create Camera <<<");
  auto camera = factory.Create<Camera>(camera_config);

  AJ_INFO(">>> Create Renderer <<<");
  auto renderer = factory.Create<Renderer>(renderer_config);

  AJ_INFO(">>> Create PostProcessors <<<");
  std::vector<Rc<PostProcessor>> post_processors;
  if (post_processor_config) {
    if (post_processor_config.IsSequence()) {
      for (int i = 0; i < post_processor_config.size(); ++i) {
        post_processors.push_back(
            factory.Create<PostProcessor>(post_processor_config[i]));
      }
    } else {
      post_processors.push_back(
          factory.Create<PostProcessor>(post_processor_config));
    }
  } else {
    AJ_INFO("No post processor");
  }

  AJ_INFO(">>> Start Rendering <<<");
  auto film = camera->CreateFilm();
  auto sampler = CreateRandomSampler();
  renderer->Render(scene.get(), camera.get(), film.get(), sampler.get());
  AJ_INFO(">>> End Rendering <<<");

  // film->WriteImage("normal.png");
  for (auto& processor : post_processors) {
    processor->Process(film.get());
  }
}

int main(int argc, char* argv[]) {
  AJ_INFO(">>> Ajisai Renderer <<<");

  run(argc, argv);

  AJ_INFO(">>> End <<<");

  return 0;
}