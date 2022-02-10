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
#include <ajisai/factory/creator/aggregate_creators.h>
#include <ajisai/factory/creator/helper.h>

AJ_BEGIN

class NativeAggregateCreatorImpl {
 public:
  static std::string Name() { return "native"; }

  static Rc<Aggregate> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    return CreateNativeAggregate();
  }
};

class PLOCAggregateCreatorImpl {
 public:
  static std::string Name() { return "ploc"; }

  static Rc<Aggregate> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    auto search_radius = node["search_radius"].as<size_t>(14);
    return CreatePLOCAggregate(search_radius);
  }
};

class EmbreeAggregateCreatorImpl {
 public:
  static std::string Name() { return "embree"; }

  static Rc<Aggregate> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    return CreateEmbreeAggregate();
  }
};

template <class TAggregateCreatorImpl>
concept AggregateCreatorImpl = requires(TAggregateCreatorImpl) {
  { TAggregateCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TAggregateCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Aggregate>>;
};

template <AggregateCreatorImpl TAggregateCreatorImpl>
class AggregateCreator : public TAggregateCreatorImpl {};

using NativeAggregateCreator = AggregateCreator<NativeAggregateCreatorImpl>;
using PLOCAggregateCreator = AggregateCreator<PLOCAggregateCreatorImpl>;
using EmbreeAggregateCreator = AggregateCreator<EmbreeAggregateCreatorImpl>;

void AddAggregateFactory(Factory<Aggregate>& factory) {
  factory.Add(NativeAggregateCreator::Name(), &NativeAggregateCreator::Create);
  factory.Add(PLOCAggregateCreator::Name(), &PLOCAggregateCreator::Create);
  factory.Add(EmbreeAggregateCreator::Name(), &EmbreeAggregateCreator::Create);
}

AJ_END