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
#include <ajisai/math/matrix4.h>

#include <yaml-cpp/yaml.h>

namespace YAML {
template <>
struct convert<aj::Vector3f> {
  static Node encode(const aj::Vector3f& rhs) {
    Node node;
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());
    return node;
  }

  static bool decode(const Node& node, aj::Vector3f& rhs) {
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
struct convert<aj::Vector2f> {
  static Node encode(const aj::Vector2f& rhs) {
    Node node;
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    return node;
  }

  static bool decode(const Node& node, aj::Vector2f& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x() = node[0].as<float>();
    rhs.y() = node[1].as<float>();
    return true;
  }
};

template <>
struct convert<aj::Matrix4f> {
  static Node encode(const aj::Matrix4f& rhs) {
    Node node;
    for (size_t i = 0; i != 15; ++i) {
      size_t col = i % 4;
      size_t row = i / 4;
      node.push_back(rhs[col][row]);
    }
    return node;
  }

  static bool decode(const Node& node, aj::Matrix4f& rhs) {
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

AJ_BEGIN

static Matrix4f GetTransformImpl(const YAML::Node& node) {
  //   if (!node["type"].IsDefined()) {
  //     AJ_ERROR("expert type node");
  //     return {};
  //   }
  const auto& type = node["type"].as<std::string>("");
  if (type == "matrix") {
    if (auto mat = node["matrix"]) {
      return mat.as<Matrix4f>();
    }

    AJ_ERROR("matrix expected");
  }

  AJ_ERROR("no valid type: {}", type);

  return {};
}

inline Matrix4f GetTransform(const YAML::Node& node) {
  if (!node.IsDefined()) return {};
  if (node.IsSequence()) {
    Matrix4f ret{};
    for (size_t i = 0; i < node.size(); ++i) {
      ret = ret * GetTransform(node[i]);
    }
    return ret;
  } else {
    return GetTransformImpl(node);
  }
}

AJ_END