#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
// #include "common/magic.h"
#include "pattern/traits.h"

namespace cm::pattern {

static constexpr size_t M_2MP_W = 1920;
static constexpr size_t M_2MP_H1 = 1080;
static constexpr size_t M_2MP_H2 = 1024;
static constexpr size_t M_8MP_W = 3840;
static constexpr size_t M_8MP_H = 2160;
static constexpr size_t M_RGB_C = 3;
static constexpr size_t M_YUV444_C = 3;
static constexpr size_t M_YUV422_C = 2;
static constexpr double M_YUV420_C = 1.5;

static constexpr double MAGIC_NUM_1INDEX6 = 1.0e6;

static constexpr double S_2_US = 1000000.0;
static constexpr double S_2_MS = 1000.0;

static const std::string RENDER_COLOR = "lightgrey";
static const std::string DEFAULT_STR = "";
static const std::string METADATA_TXT = "metadata.txt";

static constexpr size_t M_POOL_SIZE = 1'000'000;
static constexpr size_t M_POINT_SIZE = 1'000'000;

}  // namespace stoic::cm::pattern

namespace stoic::cm::pattern {

#define STOIC_WORKFLOW_ADD_TASK(T, ...)                                                   \
  namespace stoic::cm::pattern {                                                          \
  template <>                                                                             \
  struct TaskTraits<T> {                                                                  \
    template <typename U, typename O, typename... Args>                                   \
    struct Output {                                                                       \
      typedef O type;                                                                     \
    };                                                                                    \
    typedef Output<T, __VA_ARGS__>::type Subject;                                         \
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
    static inline std::string outputClassName() { return parseOutput<T, __VA_ARGS__>(); } \
    static inline std::unordered_set<std::string> inputClassNames() {                     \
      std::unordered_set<std::string> inputs;                                             \
      parseInputs<T, __VA_ARGS__>(&inputs);                                               \
      return inputs;                                                                      \
    }                                                                                     \
  };                                                                                      \
  }

struct TaskOptions {
  TaskOptions(const std::string& _name, const std::string& _deployment, const std::string& _module,
              const std::string& _submodule, const std::string& _topic,
              const std::string& _color = stoic::cm::pattern::RENDER_COLOR,
              const std::string& _node = stoic::cm::pattern::DEFAULT_STR)
      : name(_name),
        deployment(_deployment),
        module(_module),
        submodule(_submodule),
        topic(_topic),
        color(_color),
        node(_node) {}

  std::string name;
  std::string deployment;
  std::string module;
  std::string submodule;
  std::string topic;
  std::string color;
  std::string node;
};

template <typename _Derived>
class Task {
 public:
  using Derived = _Derived;

  template <typename... OptionArgs>
  Task(cm::NodeHandle& nh, OptionArgs&&... args)
      : nh_(nh), options_(std::forward<OptionArgs>(args)...) {}

  Task(cm::NodeHandle& nh, const TaskOptions& options) : nh_(nh), options_(options) {}

  template <typename Msg>
  void subscribe(const std::string& topic,
                 const boost::function<void(std::shared_ptr<Msg>)>& callback) {
    subscribers_.emplace(topic, std::move(nh_.subscribe<Msg, true>(topic, 1, callback)));
  }

  inline std::string className() const { return TaskTraits<Derived>::className(); }

  inline std::string sourceFile() const { return TaskTraits<Derived>::sourceFile(); }

  inline std::string outputClassName() const { return TaskTraits<Derived>::outputClassName(); }

  inline std::unordered_set<std::string> inputClassNames() const {
    return TaskTraits<Derived>::inputClassNames();
  }

  inline const std::string& displayName() const { return options_.name; }

  inline const std::string& deploymentName() const { return options_.deployment; }

  inline const std::string& moduleName() const { return options_.module; }

  inline const std::string& submoduleName() const { return options_.submodule; }

  inline const std::string& topicName() const { return options_.topic; }

  inline const std::string& colorName() const { return options_.color; }

  inline const std::string& nodeName() const { return options_.node; }

  cm::Subscriber* getSubscriber(const std::string& topic) {
    auto iter = subscribers_.find(topic);
    if (iter != subscribers_.end()) {
      return &iter->second;
    } else {
      return nullptr;
    }
  }

  virtual void run() {}

 private:
  cm::NodeHandle& nh_;
  TaskOptions options_;
  std::unordered_map<std::string, cm::Subscriber> subscribers_;
};

}  // namespace cm::pattern
