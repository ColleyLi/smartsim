#pragma once

#include <memory>
#include <tuple>
#include "boost/pool/object_pool.hpp"

namespace alg {

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

template <typename T>
struct ObjectPool {
  ObjectPool(const size_t& size) : pool_(size) {}

  /*
  static ObjectPool& instance() {
    static ObjectPool pool(1);
    return pool;
  }
  */

  std::shared_ptr<T> malloc() {
    std::lock_guard<std::mutex> lock(pool_mutex_);
    return std::shared_ptr<T>(pool_.malloc(), [this](T* const data) {
      std::lock_guard<std::mutex> lock(pool_mutex_);
      pool_.free(data);
    });
  }

  template <typename... Args>
  std::shared_ptr<T> calloc(Args&&... args) {
    std::lock_guard<std::mutex> lock(pool_mutex_);
    return std::shared_ptr<T>(pool_.construct(std::forward<Args>(args)...), [this](T* data) {
      std::lock_guard<std::mutex> lock(pool_mutex_);
      pool_.destroy(data);
    });
  }

 private:
  boost::object_pool<T> pool_;
  std::mutex pool_mutex_;
};

}  // namespace alg