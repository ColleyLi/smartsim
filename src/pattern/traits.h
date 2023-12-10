#pragma once

#include <string>
// #include "common/traits.hpp"

namespace cm::pattern {

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

}  // namespace stoic::cm::pattern

namespace stoic::cm::pattern {

template <typename T>
struct TaskTraits;

// template <typename T>
// struct TaskTraits {
// static inline std::string className() { return reflection<T>::fullName(); }
//
// static std::string interfaces() {
// std::stringstream ss;
// return ss.str();
// }
// };

}  // namespace cm::pattern
