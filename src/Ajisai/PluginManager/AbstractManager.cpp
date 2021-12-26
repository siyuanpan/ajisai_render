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

#include <Ajisai/PluginManager/AbstractManager.h>
#include <Ajisai/PluginManager/configure.h>
#include <dlfcn.h>

#include <filesystem>
#include <iostream>

namespace Ajisai::PluginManager {
AbstractManager::AbstractManager(
    const std::vector<std::filesystem::path>& searchPath)
    : searchPath(searchPath) {}

AbstractManager::~AbstractManager() {}

void AbstractManager::loadImpl(const std::string& plugin) {
  auto find = plugins.find(plugin);
  if (find != plugins.end()) {
    return;  // find->second(*this, plugin);
  }
  for (auto& prefix : searchPath) {
    auto path = prefix / plugin;
    path = path.concat(PLUGIN_FILENAME_SUFFIX);
    std::cout << "Loading " << path.string() << std::endl;
    void* lib = dlopen(path.string().data(), RTLD_NOW | RTLD_GLOBAL);
    if (!lib) {
      continue;
    } else {
      auto constructor =
          reinterpret_cast<Constructor>(dlsym(lib, "pluginConstructor"));
      if (constructor == nullptr) {
        std::cerr
            << "PluginManager::Manager::load(): cannot get constructor of "
               "plugin\n";
        dlclose(lib);
        return;  // nullptr;
      }

      plugins.insert({plugin, constructor});

      return;  // constructor(*this, plugin);
    }
  }

  std::cerr << "PluginManager::Manager::load(): cannot load plugin " << plugin
            << std::endl;
  return;  // nullptr;
}

void* AbstractManager::loadAndInstantiateImpl(const std::string& plugin) {
  auto find = plugins.find(plugin);
  if (find != plugins.end()) {
    return find->second(*this, plugin);
  }
  for (auto& prefix : searchPath) {
    auto path = prefix / plugin;
    path = path.concat(PLUGIN_FILENAME_SUFFIX);
    std::cout << "Loading " << path.string() << std::endl;
    void* lib = dlopen(path.string().data(), RTLD_NOW | RTLD_GLOBAL);
    if (!lib) {
      continue;
    } else {
      auto constructor =
          reinterpret_cast<Constructor>(dlsym(lib, "pluginConstructor"));
      if (constructor == nullptr) {
        std::cerr
            << "PluginManager::Manager::load(): cannot get constructor of "
               "plugin\n";
        dlclose(lib);
        return nullptr;
      }

      plugins.insert({plugin, constructor});

      return constructor(*this, plugin);
    }
  }

  std::cerr << "PluginManager::Manager::load(): cannot load plugin " << plugin
            << std::endl;
  return nullptr;
}

}  // namespace Ajisai::PluginManager