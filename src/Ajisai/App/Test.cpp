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

  {
    // RectangularMatrix<float, 2, 3> matrix{Vector3f{1.f, 2.f, 3.f},
    //                                       Vector3f{4.f, 5.f, 6.f}};
    RectangularMatrix3x3f matrix{3.4f};
    RectangularMatrix<double, 4, 3> matrixd{IdentityInit};

    Matrix<float, 3> lookat(Vector3f{0.f, 1.f, 2.f}, Vector3f{}, Vector3f{});
    // Matrix<float, 3> lookat(5.f);
    Matrix<float, 4> ll(lookat);

    Vector3f translation{5.3f, -8.9f, -10.0f};
    Vector3f target{19.0f, 29.3f, 0.0f};
    Matrix4<float> M4 =
        Matrix4<float>::lookAt(translation, target, Vector3f::xAxis());

    Matrix4<float> MT = Matrix4<float>::translation(Vector3f{1.0f, 2.f, 7.f});
    Matrix3x3<float> a(Vector3(std::sqrt(3.f) / 2.0f, 0.5f, 0.0f),
                       Vector3(-0.5f, std::sqrt(3.f) / 2.0f, 0.0f),
                       Vector3(0.0f, 0.0f, 1.0f));
    std::cout << matrix.data()[0] << std::endl;
    std::cout << matrixd << std::endl;
    std::cout << matrixd.transposed() << std::endl;
    std::cout << lookat << std::endl;
    std::cout << lookat.transposed() << std::endl;
    std::cout << ll << std::endl;
    std::cout << M4 << std::endl;
    std::cout << MT << std::endl;
    std::cout << MT.transposed() << std::endl;
    std::cout << a << std::endl;
    std::cout << a.invertedOrthogonal() * a << std::endl;
    std::cout << a.inverted() * a << std::endl;
    Matrix4x4<float> m(
        Vector4(3.0f, 5.0f, 8.0f, 4.0f), Vector4(4.0f, 4.0f, 7.0f, 3.0f),
        Vector4(7.0f, -1.0f, 8.0f, 0.0f), Vector4(9.0f, 4.0f, 5.0f, 9.0f));
    Matrix4x4<float> inverse(
        Vector4(-60 / 103.0f, 71 / 103.0f, -4 / 103.0f, 3 / 103.0f),
        Vector4(-66 / 103.0f, 109 / 103.0f, -25 / 103.0f, -7 / 103.0f),
        Vector4(177 / 412.0f, -97 / 206.0f, 53 / 412.0f, -7 / 206.0f),
        Vector4(259 / 412.0f, -185 / 206.0f, 31 / 412.0f, 27 / 206.0f));
    std::cout << m * inverse << std::endl;
    std::cout << m.inverted() * m << std::endl;

    std::cout << M4 << std::endl;
    std::cout << M4.inverted() * M4 << std::endl;
    std::cout << M4.invertedRigid() * M4 << std::endl;
  }

  {
    Matrix4f expected({4.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 7.111111f, 0.0f, 0.0f},
                      {0.0f, 0.0f, -1.9411764f, -1.0f},
                      {0.0f, 0.0f, -94.1176452f, 0.0f});
    Matrix4f actual = Matrix4f::perspectiveProjection(
        {-9.0f, 5.0f}, {7.0f, -4.0f}, 32.0f, 100.0f);
    std::cout << expected << std::endl;
    std::cout << actual << std::endl;
    std::cout << actual.transformPoint({7.0f, 4.0f, -32.0f}) << std::endl;
    std::cout << actual.transformPoint({0.0f, 0.0f, -100.0f}) << std::endl;
    std::cout << Matrix4f::perspectiveProjection(
                     {-8.0f, 4.5f}, {8.0f, -4.5f}, 32.0f,
                     std::numeric_limits<float>::infinity())
              << std::endl;
    // Rad<float> rad;
  }
}