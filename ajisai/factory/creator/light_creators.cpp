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
#include <ajisai/factory/creator/light_creators.h>
#include <ajisai/factory/creator/helper.h>

AJ_BEGIN

class DirectionalCreatorImpl {
 public:
  static std::string Name() { return "directional"; }

  static Rc<Light> Create(const YAML::Node& node,
                          const CreateFactory& factory) {
    const Vector3f dir = node["direction"].as<Vector3f>();
    auto angle = Deg<float>(node["angle"].as<float>(0));
    const auto emission = node["emission"].as<Vector3f>();
    const int32_t power = node["power"].as<int32_t>(-1);

    return CreateDirectionalLight(dir, angle, emission, power);
  }
};

class EnvmapCreatorImpl {
 public:
  static std::string Name() { return "envmap"; }

  static Rc<Light> Create(const YAML::Node& node,
                          const CreateFactory& factory) {
    auto texture = factory.Create<Texture2D>(node["texture"]);
    float rot = node["rot"].as<float>(0.f);
    float scaler = node["scaler"].as<float>(1.f);

    return CreateEnvLight(std::move(texture), rot, scaler);
  }
};

template <class TLightCreatorImpl>
concept LightCreatorImpl = requires(TLightCreatorImpl) {
  { TLightCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TLightCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Light>>;
};

template <LightCreatorImpl TLightCreatorImpl>
class LightCreator : public TLightCreatorImpl {};

using DirectionalCreator = LightCreator<DirectionalCreatorImpl>;
using EnvmapCreator = LightCreator<EnvmapCreatorImpl>;

void AddLightFactory(Factory<Light>& factory) {
  factory.Add(DirectionalCreator::Name(), &DirectionalCreator::Create);
  factory.Add(EnvmapCreator::Name(), &EnvmapCreator::Create);
}

AJ_END