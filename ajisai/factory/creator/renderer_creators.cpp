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
#include <ajisai/factory/creator/renderer_creators.h>
#include <ajisai/factory/creator/helper.h>

AJ_BEGIN

class PathTracingCreatorImpl {
 public:
  static std::string Name() { return "pt"; }

  static Rc<Renderer> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const int spp = node["spp"].as<int>();
    const int tile_size = node["tile_size"].as<int>(16);
    const int min_bounces = node["min_bounces"].as<int>(5);
    const int max_bounces = node["max_bounces"].as<int>(10);
    const float cont_prob = node["cont_prob"].as<float>(0.9);
    const bool use_mis = node["use_mis"].as<bool>(true);
    const int specular_depth = node["specular_depth"].as<int>(20);

    AJ_INFO("spp : {}", spp);
    AJ_INFO("tile_size : {}", tile_size);
    AJ_INFO("min_bounces : {}", min_bounces);
    AJ_INFO("max_bounces : {}", max_bounces);
    AJ_INFO("cont_prob : {}", cont_prob);
    AJ_INFO("use_mis : {}", use_mis);
    AJ_INFO("specular_depth : {}", specular_depth);

    PTRendererArgs args{spp,       tile_size, min_bounces,   max_bounces,
                        cont_prob, use_mis,   specular_depth};

    return CreatePTRenderer(args);
  }
};

class SPPMCreatorImpl {
 public:
  static std::string Name() { return "sppm"; }

  static Rc<Renderer> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const int forward_max_depth = node["forward_max_depth"].as<int>(8);
    const float init_radius = node["init_radius"].as<float>(0.1f);
    const int iteration_count = node["iteration_count"].as<int>(100);
    const int photons_per_iteration =
        node["photons_per_iteration"].as<int>(100000);
    const int photon_min_depth = node["photon_min_depth"].as<int>(5);
    const int photon_max_depth = node["photon_max_depth"].as<int>(10);
    const float photon_cont_prob = node["photon_cont_prob"].as<float>(0.9f);
    const float update_alpha = node["update_alpha"].as<float>(2.f / 3);

    AJ_INFO("forward_max_depth : {}", forward_max_depth);
    AJ_INFO("init_radius : {}", init_radius);
    AJ_INFO("iteration_count : {}", iteration_count);
    AJ_INFO("photons_per_iteration : {}", photons_per_iteration);
    AJ_INFO("photon_min_depth : {}", photon_min_depth);
    AJ_INFO("photon_max_depth : {}", photon_max_depth);
    AJ_INFO("photon_cont_prob : {}", photon_cont_prob);
    AJ_INFO("update_alpha : {}", update_alpha);

    SPPMRendererArgs args{
        forward_max_depth,     init_radius,      iteration_count,
        photons_per_iteration, photon_min_depth, photon_max_depth,
        photon_cont_prob,      update_alpha,
    };

    return CreateSPPMRenderer(args);
  }
};

// struct VCMRendererArgs {
//   int iteration;
//   float base_radius;
//   float radius_alpha;

//   bool use_vm;
//   bool use_vc;

//   int min_path_length;
//   int max_path_length;

//   float cont_prob;
// };

class VCMCreatorImpl {
 public:
  static std::string Name() { return "vcm"; }

  static Rc<Renderer> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const int iteration = node["iteration"].as<int>(1);
    const float radius = node["radius"].as<float>(0.003f);
    const float alpha = node["alpha"].as<float>(0.75f);
    const bool use_vm = node["use_vm"].as<bool>(true);
    const bool use_vc = node["use_vc"].as<bool>(true);
    const int min_path_length = node["min_path_length"].as<int>(0);
    const int max_path_length = node["max_path_length"].as<int>(10);
    const float cont_prob = node["cont_prob"].as<float>(0.9);

    AJ_INFO("iteration : {}", iteration);
    AJ_INFO("radius : {}", radius);
    AJ_INFO("alpha : {}", alpha);
    AJ_INFO("use_vm : {}", use_vm);
    AJ_INFO("use_vc : {}", use_vc);
    AJ_INFO("min_path_length : {}", min_path_length);
    AJ_INFO("max_path_length : {}", max_path_length);
    AJ_INFO("cont_prob : {}", cont_prob);

    VCMRendererArgs args{
        iteration, radius,          alpha,           use_vm,
        use_vc,    min_path_length, max_path_length, cont_prob,
    };

    return CreateVCMRenderer(args);
  }
};

template <class TRendererCreatorImpl>
concept RendererCreatorImpl = requires(TRendererCreatorImpl) {
  { TRendererCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TRendererCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Renderer>>;
};

template <RendererCreatorImpl TRendererCreatorImpl>
class RendererCreator : public TRendererCreatorImpl {};

using PathTracingCreator = RendererCreator<PathTracingCreatorImpl>;
using SPPMCreator = RendererCreator<SPPMCreatorImpl>;
using VCMCreator = RendererCreator<VCMCreatorImpl>;

void AddRendererFactory(Factory<Renderer>& factory) {
  factory.Add(PathTracingCreator::Name(), &PathTracingCreator::Create);
  factory.Add(SPPMCreator::Name(), &SPPMCreator::Create);
  factory.Add(VCMCreator::Name(), &VCMCreator::Create);
}

AJ_END