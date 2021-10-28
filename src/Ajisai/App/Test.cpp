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
}