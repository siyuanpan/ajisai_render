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
#include <ajisai/factory/creator/filter_creators.h>

AJ_BEGIN

class BoxFilterCreatorImpl {
 public:
  static std::string Name() { return "box"; }

  static Rc<Filter> Create(const YAML::Node& node,
                           const CreateFactory& factory) {
    auto xwidth = node["xwidth"].as<float>(0.5f);
    auto ywidth = node["ywidth"].as<float>(0.5f);

    return CreateBoxFilter(xwidth, ywidth);
  }
};

class GaussianFilterCreatorImpl {
 public:
  static std::string Name() { return "gaussian"; }

  static Rc<Filter> Create(const YAML::Node& node,
                           const CreateFactory& factory) {
    auto xwidth = node["xwidth"].as<float>(2.f);
    auto ywidth = node["ywidth"].as<float>(2.f);
    auto alpha = node["alpha"].as<float>(2.f);

    return CreateGaussianFilter(xwidth, ywidth, alpha);
  }
};

class MitchellFilterCreatorImpl {
 public:
  static std::string Name() { return "mitchell"; }

  static Rc<Filter> Create(const YAML::Node& node,
                           const CreateFactory& factory) {
    auto xwidth = node["xwidth"].as<float>(2.f);
    auto ywidth = node["ywidth"].as<float>(2.f);
    auto B = node["B"].as<float>(1.f / 3.f);
    auto C = node["C"].as<float>(1.f / 3.f);

    return CreateMitchellFilter(xwidth, ywidth, B, C);
  }
};

template <class TFilterCreatorImpl>
concept FilterCreatorImpl = requires(TFilterCreatorImpl) {
  { TFilterCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TFilterCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Filter>>;
};

template <FilterCreatorImpl TFilterCreatorImpl>
class FilterCreator : public TFilterCreatorImpl {};

using BoxFilterCreator = FilterCreator<BoxFilterCreatorImpl>;
using GaussianFilterCreator = FilterCreator<GaussianFilterCreatorImpl>;
using MitchellFilterCreator = FilterCreator<MitchellFilterCreatorImpl>;

void AddFilterFactory(Factory<Filter>& factory) {
  factory.Add(BoxFilterCreator::Name(), &BoxFilterCreator::Create);
  factory.Add(GaussianFilterCreator::Name(), &GaussianFilterCreator::Create);
  factory.Add(MitchellFilterCreator::Name(), &MitchellFilterCreator::Create);
}

AJ_END