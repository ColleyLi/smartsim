#pragma once

#include <boost/bind/bind.hpp>
#include <boost/functional/hash.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "pattern/observer.hpp"
#include "pattern/task.hpp"

namespace cm::pattern {

template <typename T>
struct IdHasher {
  std::size_t operator()(const T& node) const { return std::hash<void*>()(node.id); }
  std::size_t operator()(const T* node) const { return std::hash<void*>()(node->id); }
};

template <typename T>
struct IdEquals {
  std::size_t operator()(const T& lhs, const T& rhs) const { return lhs.id == rhs.id; }
  std::size_t operator()(const T* lhs, const T* rhs) const { return lhs->id == rhs->id; }
};

template <typename T, typename U>
struct PairHasher {
  std::size_t operator()(const std::pair<T, U>& pair) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, pair.first);
    boost::hash_combine(seed, pair.second);
    return seed;
  }
};

template <typename T, typename U>
struct PairEquals {
  std::size_t operator()(const std::pair<T, U>& lhs, const std::pair<T, U>& rhs) const {
    return lhs.first == rhs.first && lhs.second == rhs.second;
  }
};

using PairVoidConstPtrSet =
    std::unordered_set<std::pair<const void*, const void*>, PairHasher<const void*, const void*>,
                       PairEquals<const void*, const void*>>;

struct TaskNode {
  void* id;
  std::string class_name;
  std::string source_file;
  std::string output_class_name;
  std::unordered_set<std::string> input_class_names;
  std::string display_name;
  std::string deployment;
  std::string module;
  std::string submodule;
  std::string topic;
  std::string color;
  std::unordered_set<const TaskNode*, IdHasher<TaskNode>, IdEquals<TaskNode>> downstreams;
};

using TaskNodeConstPtrSet =
    std::unordered_set<const TaskNode*, IdHasher<TaskNode>, IdEquals<TaskNode>>;

template <typename T>
using TaskNodeConstPtrMap =
    std::unordered_map<const TaskNode*, T, IdHasher<TaskNode>, IdEquals<TaskNode>>;

class Workflow {
 public:
  Workflow() {}

  template <typename Upstream, typename Downstream>
  void addLink(Upstream* const upstream, Downstream* const downstream,
               const std::string& deployment) {
    TaskNode& parent = addTask(upstream);
    TaskNode& child = addTask(downstream);
    parent.downstreams.emplace(&child);
    if (parent.deployment.find(deployment) != std::string::npos &&
        child.deployment.find(deployment) != std::string::npos) {
      upstream->addDownstream(downstream);
    } else if (parent.deployment.find(deployment) == std::string::npos &&
               child.deployment.find(deployment) != std::string::npos) {
      downstream->template subscribe<typename TaskTraits<Upstream>::Subject>(
          upstream->topicName(),
          boost::bind(
              static_cast<void (
                  ::stoic::cm::pattern::Observer<typename TaskTraits<Upstream>::Subject>::*)(
                  std::shared_ptr<typename TaskTraits<Upstream>::Subject>, const std::string&)>(
                  &::stoic::cm::pattern::Observer<
                      typename TaskTraits<Upstream>::Subject>::callback),
              downstream, boost::placeholders::_1, upstream->topicName()));
    } else
      return;
  }

  const std::unordered_map<void*, TaskNode>& getTasks() const { return tasks_; }

 private:
  template <typename T>
  TaskNode& addTask(T* task) {
    auto iter = tasks_.find(task);
    if (iter == tasks_.end()) {
      iter = tasks_
                 .emplace(task, TaskNode{task,
                                         task->className(),
                                         task->sourceFile(),
                                         task->outputClassName(),
                                         task->inputClassNames(),
                                         task->displayName(),
                                         task->deploymentName(),
                                         task->moduleName(),
                                         task->submoduleName(),
                                         task->topicName(),
                                         task->colorName(),
                                         {}})
                 .first;
    }
    return iter->second;
  }

 private:
  std::unordered_map<void*, TaskNode> tasks_;
};

}  // namespace cm::pattern
