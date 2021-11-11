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

#ifndef AJISAI_PLUGINMANAGER_MANAGER_H_
#define AJISAI_PLUGINMANAGER_MANAGER_H_

#include <Ajisai/PluginManager/AbstractManager.h>

#include <iostream>
#include <memory>

namespace Ajisai::PluginManager {
template <class T>
class Manager : public AbstractManager {
 public:
  //   static const int Version;

  explicit Manager() : AbstractManager{T::pluginSearchPaths()} {}

  //   Manager(const Manager<T>&) = delete;
  //   Manager(Manager<T>&&) = delete;
  //   Manager<T>& operator=(const Manager<T>&) = delete;
  //   Manager<T>& operator=(Manager<T>&&) = delete;

  T* load(const std::string& plugin) {
    // auto tmp = loadImpl(plugin);
    // std::cout << tmp << std::endl;
    // std::cout << reinterpret_cast<T*>(tmp) << std::endl;
    // std::cout << static_cast<T*>(tmp) << std::endl;
    // return static_cast<T*>(tmp);
    return static_cast<T*>(loadImpl(plugin));
    // return reinterpret_cast<T*>(loadImpl(plugin));
    //     void* lib = dlopen(plugin.data(), RTLD_NOW | RTLD_GLOBAL);
    //     if (!lib) {
    //       std::cerr << "PluginManager::Manager::load(): cannot load plugin "
    //                 << plugin << std::endl;
    //     }
  }
};

// #define AJISAI_PLUGIN_REGISTER(name, className, interface)                  \
//   extern "C" AJISAI_API_EXPORT void* pluginCreate(                          \
//       Ajisai::PluginManager::Manager& manager, const std::string& plugin);  \
//   extern "C" AJISAI_API_EXPORT void* pluginCreate(                          \
//       Ajisai::PluginManager::Manager& manager, const std::string& plugin) { \
//     return new className{manager, plugin};                                  \
//   }                                                                         \
//   extern "C" AJISAI_API_EXPORT const char* pluginInterface();               \
//   extern "C" AJISAI_API_EXPORT const char* pluginInterface() {              \
//     return interface;                                                       \
//   }

}  // namespace Ajisai::PluginManager

#endif