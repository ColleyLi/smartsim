#pragma once

#include <string>
#include "cxxabi.h"

template <typename T>
struct TaskTraits;

template <typename T>
struct TaskTraits2;

template <typename T>
struct reflection {
  static std::string fullName() {
    int status;
    char* demangled = abi::__cxa_demangle(typeid(T).name(), 0, 0, &status);
    std::string class_name(demangled);
    free(demangled);  // PRQA S 5211
    return class_name;
  }
};
