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
#include <ajisai/factory/creator/material_creators.h>
#include <ajisai/factory/creator/helper.h>

AJ_BEGIN

class DiffuseCreatorImpl {
 public:
  static std::string Name() { return "diffuse"; }

  static Rc<Material> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    // AJ_DEBUG("type : {} , albedo.type : {}", node["type"].as<std::string>(),
    //          node["albedo"]["type"].as<std::string>());
    const auto albedo = factory.Create<Texture2D>(node["albedo"]);

    return CreateDiffuse(std::move(albedo));
  }
};

class PlasticCreatorImpl {
 public:
  static std::string Name() { return "plastic"; }

  static Rc<Material> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const auto albedo = factory.Create<Texture2D>(node["albedo"]);
    const auto ior = node["ior"].as<float>(1.5);
    const auto thickness = node["thickness"].as<float>(1);
    const auto sigma_a = node["sigma_a"].as<float>(0);

    return CreatePlastic(std::move(albedo), ior, thickness, sigma_a);
  }
};

class MetalCreatorImpl {
 public:
  static std::string Name() { return "metal"; }

  static Rc<Material> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const auto color = factory.Create<Texture2D>(node["color"]);
    const auto k = factory.Create<Texture2D>(node["k"]);
    const auto eta = factory.Create<Texture2D>(node["eta"]);
    const auto uroughness = node["uroughness"].as<float>(0.1f);
    const auto vroughness = node["vroughness"].as<float>(0.1f);

    return CreateMetal(std::move(color), std::move(k), std::move(eta),
                       uroughness, vroughness);
  }
};

class MirrorCreatorImpl {
 public:
  static std::string Name() { return "mirror"; }

  static Rc<Material> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const auto kr = factory.Create<Texture2D>(node["kr"]);

    return CreateMirror(std::move(kr));
  }
};

class DisneyCreatorImpl {
 public:
  static std::string Name() { return "disney"; }

  static Rc<Material> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const auto base_color = factory.Create<Texture2D>(node["base_color"]);
    const auto subsurface = factory.Create<Texture2D>(node["subsurface"]);
    const auto metallic = factory.Create<Texture2D>(node["metallic"]);
    const auto specular = factory.Create<Texture2D>(node["specular"]);
    const auto specular_tint = factory.Create<Texture2D>(node["specular_tint"]);
    const auto roughness = factory.Create<Texture2D>(node["roughness"]);
    const auto anisotropic = factory.Create<Texture2D>(node["anisotropic"]);
    const auto sheen = factory.Create<Texture2D>(node["sheen"]);
    const auto sheen_tint = factory.Create<Texture2D>(node["sheen_tint"]);
    const auto clearcoat = factory.Create<Texture2D>(node["clearcoat"]);
    const auto clearcoat_gloss =
        factory.Create<Texture2D>(node["clearcoat_gloss"]);

    return CreateDisney(
        std::move(base_color), std::move(subsurface), std::move(metallic),
        std::move(specular), std::move(specular_tint), std::move(roughness),
        std::move(anisotropic), std::move(sheen), std::move(sheen_tint),
        std::move(clearcoat), std::move(clearcoat_gloss));
  }
};

class GlassCreatorImpl {
 public:
  static std::string Name() { return "glass"; }

  static Rc<Material> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    Rc<Texture2D> color_reflection;
    Rc<Texture2D> color_refraction;

    if (node["color_reflection"].IsDefined()) {
      color_reflection = factory.Create<Texture2D>(node["color_reflection"]);
    } else {
      color_reflection = CreateConstant2DTexture(Spectrum{1.f});
    }

    if (node["color_refraction"].IsDefined()) {
      color_refraction = factory.Create<Texture2D>(node["color_refraction"]);
    } else {
      color_refraction = CreateConstant2DTexture(Spectrum{1.f});
    }

    const auto ior = factory.Create<Texture2D>(node["ior"]);

    return CreateGlass(std::move(color_reflection), std::move(color_refraction),
                       std::move(ior));
  }
};

template <class TMaterialCreatorImpl>
concept MaterialCreatorImpl = requires(TMaterialCreatorImpl) {
  { TMaterialCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TMaterialCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Material>>;
};

template <MaterialCreatorImpl TMaterialCreatorImpl>
class MaterialCreator : public TMaterialCreatorImpl {};

using DiffuseCreator = MaterialCreator<DiffuseCreatorImpl>;
using PlasticCreator = MaterialCreator<PlasticCreatorImpl>;
using MetalCreator = MaterialCreator<MetalCreatorImpl>;
using MirrorCreator = MaterialCreator<MirrorCreatorImpl>;
using DisneyCreator = MaterialCreator<DisneyCreatorImpl>;
using GlassCreator = MaterialCreator<GlassCreatorImpl>;

void AddMaterialFactory(Factory<Material>& factory) {
  factory.Add(DiffuseCreator::Name(), &DiffuseCreator::Create);
  factory.Add(PlasticCreator::Name(), &PlasticCreator::Create);
  factory.Add(MetalCreator::Name(), &MetalCreator::Create);
  factory.Add(MirrorCreator::Name(), &MirrorCreator::Create);
  factory.Add(DisneyCreator::Name(), &DisneyCreator::Create);
  factory.Add(GlassCreator::Name(), &GlassCreator::Create);
}

AJ_END