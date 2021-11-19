#include <Ajisai/Core/Random.h>
// #include <Ajisai/Math/Color.h>
#include <Ajisai/Math/Math.h>
#include <Ajisai/PluginManager/Manager.h>
#include <Ajisai/Util/Ptr.h>

#include <fstream>
#include <iostream>
#include <ryml/ryml.hpp>
#include <sstream>
#include <streambuf>
#include <string>

void split(const std::string& s, char delim, std::vector<std::string>& elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
}

class A {
 public:
  A() {}

  A(int i, int j) {}

  virtual ~A() { std::cout << "~A call\n"; }

 private:
  int* t1;
  int* t2;
};

class B : public A {
 public:
  B() {}

  virtual ~B() { std::cout << "~B call\n"; }
  // virtual std::string name() = 0;
  int spp = 16;
};

class C : public B {
 public:
  C() {}
  // virtual std::string name() override { return "C"; }
  virtual ~C() { std::cout << "~C call\n"; }

 private:
  int spp = 16;
  int minDepth = 5, maxDepth = 16;
};

int main(int argc, char** argv) {
  using namespace Ajisai;
  using namespace Ajisai::Core;
  using namespace Ajisai::Math;

  Vector3i v{1, 2, 3};
  std::cout << v[0] << " " << v[1] << " " << v[2] << std::endl;
  std::cout << (v * 2)[0] << " " << (v * 2)[1] << " " << (v * 2)[2]
            << std::endl;
  std::cout << (2 * v)[0] << " " << (2 * v)[1] << " " << (2 * v)[2]
            << std::endl;

  const char filename[] = "./scene.yml";

  std::ifstream t(filename);
  std::string contents;

  t.seekg(0, std::ios::end);
  std::cout << t.tellg() << std::endl;
  contents.reserve(t.tellg());
  t.seekg(0, std::ios::beg);

  contents.assign(std::istreambuf_iterator<char>(t),
                  std::istreambuf_iterator<char>());

  ryml::Tree tree = ryml::parse(ryml::to_csubstr(contents.c_str()));

  std::cout << tree["camera"]["Perspective"]["fov"].val() << std::endl;
  std::cout << tree["camera"]["Perspective"]["transform"]["origin"][0].val()
            << std::endl;
  auto root = tree.rootref();
  for (ryml::NodeRef n : tree["camera"]["Perspective"]["res"].children()) {
    std::cout << n.val() << std::endl;
  }

  std::string face("9//12");
  std::vector<std::string> tmp;
  split(face, '/', tmp);
  for (auto tt : tmp) {
    std::cout << tt << std::endl;
  }

  PCG32 pcg32;
  int count = 20;
  while (count--) {
    std::cout << pcg32.next_double() << std::endl;
  }

  auto boolV = Vector3i(10, 20, 3) < Vector3i(0, 3, 5);
  std::cout << (lerp(Vector3i(1, 1, 1), Vector3i(2, 2, 2), boolV))[0]
            << std::endl;
  std::cout << (lerp(Vector3i(1, 1, 1), Vector3i(2, 2, 2), boolV))[1]
            << std::endl;
  std::cout << (lerp(Vector3i(1, 1, 1), Vector3i(2, 2, 2), boolV))[2]
            << std::endl;

  //   Color3<float> color(Vector3f{0.5, 0.2, 0.3});
  auto color = Math::Color3<float>::fromSrgb(Vector3f{0.5, 0.1, 0.2});
  //   // {0.5, 0.2, 0.3};
  std::cout << color.r() << std::endl;
  std::cout << color.g() << std::endl;
  std::cout << color.b() << std::endl;

  C* c = new C();
  B* b = static_cast<B*>(c);
  A* a = static_cast<A*>(c);
  void* vc = c;
  B* vb = static_cast<B*>(vc);
  A* va = static_cast<A*>(vc);
  std::cout << c << std::endl;
  std::cout << b << std::endl;
  // std::cout << a << std::endl;
  // std::cout << static_cast<B*>(a) << std::endl;
  std::cout << vc << std::endl;
  std::cout << vb << std::endl;
  // std::cout << va << std::endl;
  // std::cout << static_cast<B*>(va) << std::endl;

  using Ajisai::Util::Ptr;

  Ptr<A> pa(new A());
  Ptr<B> pb(new B());
  pa = std::move(pb);
  // auto pa = Ptr<A>(1, 2);

  // PluginManager::Manager<int> manager;

  Bounds2i ba{{3, 2}, {5, 4}}, bb{{2, 2}, {3, 3}};

  auto bc = join(ba, bb);
  std::cout << "c.min.x " << bc.min().x() << "\n"
            << "c.min.y " << bc.min().y() << "\n"
            << "c.max.x " << bc.max().x() << "\n"
            << "c.max.y " << bc.max().y() << "\n";

  BoolVector<19> bvec(0xff, 0xff, 0x04);
  std::cout << "all : " << bvec.all() << std::endl;

  {
    Bounds2i a({34, 23}, {47, 30});
    Bounds2i b{{30, 25}, {35, 105}};
    // Bounds2i c{{34, 25}, {35, 30}};
    Bounds2i d{{130, -15}, {130, -15}};
    std::cout << "intersects : " << intersects(a, b) << std::endl;
    std::cout << "intersects : " << intersects(b, a) << std::endl;
    auto c = intersect(b, a);
    std::cout << "c.min.x " << c.min().x() << "\n"
              << "c.min.y " << c.min().y() << "\n"
              << "c.max.x " << c.max().x() << "\n"
              << "c.max.y " << c.max().y() << "\n";
  }
}