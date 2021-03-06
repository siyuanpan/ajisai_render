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

#ifndef AJISAI_PLUGINMANAGER_ABSTRACTMANAGER_H_
#define AJISAI_PLUGINMANAGER_ABSTRACTMANAGER_H_

#include <Ajisai/Ajisai.h>
#include <Ajisai/configure.h>
// #include <Ajisai/PluginManager/Common.h>
// #include <Ajisai/PluginManager/AbstractPlugin.h>
#include <Ajisai/Util/Macros.h>

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace Ajisai::PluginManager {

class AJISAI_API AbstractManager {
 public:
  typedef void* (*Constructor)(AbstractManager&, const std::string&);

  AbstractManager(const AbstractManager&) = delete;
  AbstractManager(AbstractManager&&) = delete;
  AbstractManager& operator=(const AbstractManager&) = delete;
  AbstractManager& operator=(AbstractManager&&) = delete;

 protected:
  explicit AbstractManager(const std::vector<std::filesystem::path>&);
  ~AbstractManager();

  void loadImpl(const std::string& plugin);
  void* loadAndInstantiateImpl(const std::string& plugin);

  std::vector<std::filesystem::path> searchPath;
  std::map<std::string, Constructor> plugins;
};

#define AJISAI_PLUGIN_REGISTER(name, className, interface) \
  extern "C" AJISAI_API void* pluginConstructor(           \
      Ajisai::PluginManager::AbstractManager& manager,     \
      const std::string& plugin);                          \
  extern "C" AJISAI_API void* pluginConstructor(           \
      Ajisai::PluginManager::AbstractManager& manager,     \
      const std::string& plugin) {                         \
    return new className{manager, plugin};                 \
  }                                                        \
  extern "C" AJISAI_API const char* pluginInterface();     \
  extern "C" AJISAI_API const char* pluginInterface() { return interface; }

}  // namespace Ajisai::PluginManager

#endif