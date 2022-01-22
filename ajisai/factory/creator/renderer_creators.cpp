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

void AddRendererFactory(Factory<Renderer>& factory) {
  factory.Add(PathTracingCreator::Name(), &PathTracingCreator::Create);
}

AJ_END