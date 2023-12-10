#pragma once

#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include "traits.h"

namespace smartsil::app::vtd_integration {
template <typename T>
struct TaskTraits;

#define WORKFLOW_ADD_TASK(T, ...)                                                         \
  namespace smartsil::app::vtd_integration {                                              \
  template <>                                                                             \
  struct TaskTraits<T> {                                                                  \
    template <typename U, typename O, typename... Args>                                   \
    struct Output {                                                                       \
      typedef O type;                                                                     \
    };                                                                                    \
    static inline std::string className() { return reflection<T>::fullName(); }           \
    static inline std::string sourceFile() {                                              \
      return std::string(__FILE__).substr(sizeof(STOIC_SOURCE_DIR));                      \
    }                                                                                     \
    template <typename U, typename O, typename... Args>                                   \
    static inline std::string parseOutput() {                                             \
      return reflection<O>::fullName();                                                   \
    }                                                                                     \
    template <typename U, typename O>                                                     \
    static inline void parseInputs(std::unordered_set<std::string>*) {}                   \
    template <typename U, typename O, typename I, typename... Args>                       \
    static inline void parseInputs(std::unordered_set<std::string>* const inputs) {       \
      inputs->emplace(reflection<I>::fullName());                                         \
      parseInputs<U, O, Args...>(inputs);                                                 \
    }                                                                                     \
    static inline std::unordered_set<std::string> inputClassNames() {                     \
      std::unordered_set<std::string> inputs;                                             \
      return inputs;                                                                      \
    }                                                                                     \
  };                                                                                      \
  }

struct TaskOptions {
  TaskOptions(const std::string& _name, const std::string& _deployment, const std::string& _module)
      : name(_name),
        deployment(_deployment),
        module(_module){}

  std::string name;
  std::string deployment;
  std::string module;
};

template <typename _Derived>
class Task {
 public:
  using Derived = _Derived;

  template<typename... OptionArgs>
  Task(OptionArgs &&... args)
      : options_(std::forward<OptionArgs>(args)...) {}

  Task(const TaskOptions &options) : options_(options) {}

  inline const std::string& displayName() const { return options_.name; }

  inline const std::string& deploymentName() const { return options_.deployment; }

  inline const std::string& moduleName() const { return options_.module; }

  virtual void run() {}

 private:
  TaskOptions options_;

};

} //namespace smartsil::app::vtd_integration