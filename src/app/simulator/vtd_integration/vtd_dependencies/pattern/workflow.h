#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <future>
#include <syscall.h>
#include "boost/pool/object_pool.hpp"
#include "task.hpp"
#include <unordered_map>
#include <unordered_set>

#include "method.h"

namespace smartsil::app::vtd_integration {

struct TaskNode {
  std::string display_name;
  std::string deployment;
  std::string module;
};

class Workflow {
 public:
  Workflow() {}

  template <typename T>
  void addMethod(T* const task, const std::string &deployment) {
    TaskNode &node = addTask(task);
    if (node.deployment.find(deployment) != std::string::npos ||
        node.deployment.find("ALL") != std::string::npos) {
      MethodsBase::RegisterMethod(deployment, task, &T::run);

      std::thread thread([&]() {
        auto it = MethodsBase::g_methodRegistry.find(deployment);
        if (it != MethodsBase::g_methodRegistry.end()) {
          std::function<void()> method = it->second;
          method();
        }

        std::thread::id id = std::this_thread::get_id();
        std::cout << "user thread id: " << id << ", kernel thread id: " << syscall(SYS_gettid) << std::endl;
        std::cout << "---------method: [" << node.display_name << "] -----------" << std::endl;
      });

      if (thread.joinable())
        thread.join();
    }
  }

  const std::unordered_map<void*, TaskNode>& getTasks() const { return tasks_; }

 private:
  template <typename T>
  TaskNode& addTask(T *task) {
    auto iter = tasks_.find(task);
    if (iter == tasks_.end()) {
        iter = tasks_.emplace(task, TaskNode{task->displayName(),
                                                  task->deploymentName(),
                                                  task->moduleName()})
            .first;
    }
    return iter->second;
  }
 private:
  std::unordered_map<void*, TaskNode> tasks_;

};

} // namespace smartsil::app::vtd_integration
