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

void AddMaterialFactory(Factory<Material>& factory) {
  factory.Add(DiffuseCreator::Name(), &DiffuseCreator::Create);
}

AJ_END