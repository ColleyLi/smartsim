#pragma once

#include <string>
#include <unordered_map>
#include <functional>

namespace smartsil::app {

using MethodRegistry = std::unordered_map<std::string, std::function<void()>>;

class MethodsBase {
 public:
  template <class T>
  static void RegisterMethod(const std::string& name, T* instance, void (T::*method)()) {
    g_methodRegistry[name] = [instance, method]() { (instance->*method)(); };
  }

  static MethodRegistry g_methodRegistry;
};

MethodRegistry MethodsBase::g_methodRegistry;

} // namespace smartsil::app
