#include <Ajisai/Core/Random.h>
// #include <Ajisai/Math/Color.h>
#include <Ajisai/Math/Math.h>

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
}