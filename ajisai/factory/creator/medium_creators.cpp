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
#include <ajisai/factory/creator/medium_creators.h>
#include <ajisai/factory/creator/helper.h>

AJ_BEGIN

class VoidMediumCreatorImpl {
 public:
  static std::string Name() { return "void"; }

  static Rc<Medium> Create(const YAML::Node& node,
                           const CreateFactory& factory) {
    return CreateVoidMedium();
  }
};

template <class TMediumCreatorImpl>
concept MediumCreatorImpl = requires(TMediumCreatorImpl) {
  { TMediumCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TMediumCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Medium>>;
};

template <MediumCreatorImpl TMediumCreatorImpl>
class MediumCreator : public TMediumCreatorImpl {};

using VoidMediumCreator = MediumCreator<VoidMediumCreatorImpl>;

void AddMediumFactory(Factory<Medium>& factory) {
  factory.Add(VoidMediumCreator::Name(), &VoidMediumCreator::Create);
}

AJ_END