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
#include <ajisai/factory/creator/camera_creators.h>
#include <ajisai/factory/creator/helper.h>
#include <ajisai/math/angle.h>

AJ_BEGIN

class ThinLensCreatorImpl {
 public:
  static std::string Name() { return "thin_lens"; }

  static Rc<Camera> Create(const YAML::Node& node,
                           const CreateFactory& factory) {
    const auto resolution = node["resolution"].as<Vector2f>();
    Rc<Filter> film_filter;
    if (auto filter = node["filter"]) {
      film_filter = factory.Create<Filter>(filter);
    } else {
      AJ_INFO("Use Box filter");
      film_filter = CreateBoxFilter(0.5f, 0.5f);
    }

    AJ_INFO("resolution ({}, {})", resolution.x(), resolution.y());

    auto position = node["position"].as<Vector3f>();
    auto look_at = node["look_at"].as<Vector3f>();
    auto up = node["up"].as<Vector3f>();

    auto fov = Deg<float>(node["fov"].as<float>());
    auto lens_radius = node["lens_radius"].as<float>(0);
    auto focal_distance = node["focal_distance"].as<float>(1);

    AJ_INFO("aspect_ratio : {}", resolution.AspectRatio());
    AJ_INFO("fov : {}", (float)fov);
    AJ_INFO("lens_radius : {}", (float)lens_radius);
    AJ_INFO("focal_distance : {}", (float)focal_distance);

    return CreateThinLensCamera(film_filter, resolution, position, look_at, up,
                                (float)Rad<float>(fov), lens_radius,
                                focal_distance);
  }
};

template <class TCameraCreatorImpl>
concept CameraCreatorImpl = requires(TCameraCreatorImpl) {
  { TCameraCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TCameraCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Camera>>;
};

template <CameraCreatorImpl TCameraCreatorImpl>
class CameraCreator : public TCameraCreatorImpl {};

using ThinLensCreator = CameraCreator<ThinLensCreatorImpl>;

void AddCameraFactory(Factory<Camera>& factory) {
  factory.Add(ThinLensCreator::Name(), &ThinLensCreator::Create);
}

AJ_END