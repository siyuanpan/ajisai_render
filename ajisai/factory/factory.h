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
#pragma once
#include <ajisai/ajisai.h>
#include <ajisai/utility/log.h>
#include <ajisai/core/geometry/geometry.h>
#include <ajisai/core/material/material.h>
#include <ajisai/core/texture2d/texture2d.h>
#include <ajisai/core/primitive/primitive.h>

#include <yaml-cpp/yaml.h>

AJ_BEGIN

struct Scene {};

class CreateFactory;

template <class T>
class Factory {
  using CreateFunc = Rc<T> (*)(const YAML::Node&, const CreateFactory&);

 public:
  explicit Factory(const std::string_view& name);

  template <class... Args>
  Rc<T> Create(const std::string& type, const YAML::Node& node,
               const CreateFactory& factory, Args&&... args) const;

  void Add(const std::string& name, CreateFunc);

 private:
  // using CreateFunc = std::function<Rc<T>(const YAML::Node&,
  // CreateFactory&)>;
  std::unordered_map<std::string, CreateFunc> table;

  std::unordered_map<std::string, CreateFunc>& Map() { return table; }

  const std::unordered_map<std::string, CreateFunc>& Map() const {
    return table;
  }

  std::string name_;
};

class AJISAI_API CreateFactory {
 public:
  CreateFactory();

  template <class T, class... Args>
  Rc<T> Create(const YAML::Node& node, Args&&... args) const;

  template <class T>
  Factory<T>& GetFactory() noexcept;

  template <class T>
  const Factory<T>& GetFactory() const noexcept;

 private:
  template <class... Types>
  using FactoryTuple = std::tuple<Factory<Types>...>;

  FactoryTuple<Scene, Primitive, Geometry, Material, Texture2D> factory_tuple_;
};

template <class T>
Factory<T>::Factory(const std::string_view& name) : name_(name) {}

template <class T>
template <class... Args>
Rc<T> Factory<T>::Create(const std::string& type, const YAML::Node& node,
                         const CreateFactory& factory, Args&&... args) const {
  const auto it = Map().find(type);
  return it->second(node, factory, std::forward<Args>(args)...);
}

template <class T>
void Factory<T>::Add(const std::string& name, CreateFunc func) {
  Map()[name] = func;
}

template <class T, class... Args>
Rc<T> CreateFactory::Create(const YAML::Node& node, Args&&... args) const {
  const auto& type_name = node["type"].as<std::string>();
  AJ_DEBUG("type name : {}", type_name);
  return GetFactory<T>().Create(type_name, node, *this,
                                std::forward<Args>(args)...);
}

template <class T>
Factory<T>& CreateFactory::GetFactory() noexcept {
  return std::get<Factory<T>>(factory_tuple_);
}

template <class T>
const Factory<T>& CreateFactory::GetFactory() const noexcept {
  return std::get<Factory<T>>(factory_tuple_);
}

AJ_END