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
#include <ajisai/factory/creator/scene_creators.h>

AJ_BEGIN

class DefaultSceneCreatorImpl {
 private:
  struct Args {
    std::vector<Rc<Primitive>> primitives;
  };

 public:
  static std::string Name() { return "default"; }

  static Rc<Scene> Create(const YAML::Node& node,
                          const CreateFactory& factory) {
    if (auto primitives = node["primitives"]) {
      AJ_INFO("create {} primitives", primitives.size());
      Args args{};

      for (size_t i = 0; i < primitives.size(); ++i) {
        auto primitive = factory.Create<Primitive>(primitives[i]);
        args.primitives.push_back(primitive);
      }
    }
    return RcNew<Scene>();
  }
};

template <class TSceneCreatorImpl>
concept SceneCreatorImpl = requires(TSceneCreatorImpl) {
  { TSceneCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TSceneCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Scene>>;
};

template <SceneCreatorImpl TSceneCreatorImpl>
class SceneCreator : public TSceneCreatorImpl {};

using DefaultSceneCreator = SceneCreator<DefaultSceneCreatorImpl>;

void AddSceneFactory(Factory<Scene>& factory) {
  factory.Add(DefaultSceneCreator::Name(), &DefaultSceneCreator::Create);
}

AJ_END