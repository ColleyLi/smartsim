#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include "pattern/workflow.hpp"

namespace cm::pattern {

class WorkflowVisualizer {
 public:
  WorkflowVisualizer() {}

  struct Graph {
    Graph(const std::string& _name) : id(this), name(_name) {}

    void* id;
    std::string name;
    std::unordered_map<void*, const TaskNode*> tasks;
    std::unordered_map<std::string, std::shared_ptr<Graph>> subgraphs;

    std::unordered_set<std::string> getInputClassNames(const bool& inclusive = true) const {
      std::unordered_set<std::string> names;
      if (inclusive) {
        for (auto iter = tasks.begin(); iter != tasks.end(); ++iter) {
          for (const std::string& name : iter->second->input_class_names) {
            names.emplace(name);
          }
        }
      }
      for (auto iter = subgraphs.begin(); iter != subgraphs.end(); ++iter) {
        std::unordered_set<std::string> subgraph_names = iter->second->getInputClassNames(true);
        names.insert(subgraph_names.begin(), subgraph_names.end());
      }
      return names;
    }

    std::unordered_set<std::string> getOutputClassNames(const bool& inclusive = true) const {
      std::unordered_set<std::string> names;
      if (inclusive) {
        for (auto iter = tasks.begin(); iter != tasks.end(); ++iter) {
          names.emplace(iter->second->output_class_name);
        }
      }
      for (auto iter = subgraphs.begin(); iter != subgraphs.end(); ++iter) {
        std::unordered_set<std::string> subgraph_names = iter->second->getOutputClassNames(true);
        names.insert(subgraph_names.begin(), subgraph_names.end());
      }
      return names;
    }

    std::string getColor(const bool& inclusive = true) const {
      std::stringstream ss;
      TaskNodeConstPtrSet all_tasks = getTasks(inclusive);
      for (auto iter = all_tasks.begin(); iter != all_tasks.end(); ++iter) {
        if (this->name == (*iter)->module || this->name == (*iter)->submodule) {
          ss << (*iter)->color;
          break;
        } else {
          continue;
        }
      }
      return ss.str();
    }

    std::unordered_set<std::string> getOutputClassNames(const TaskNodeConstPtrSet& downstream_tasks,
                                                        const bool& inclusive = true) const {
      std::unordered_set<std::string> names;
      TaskNodeConstPtrSet all_tasks = getTasks(inclusive);
      for (auto iter = all_tasks.begin(); iter != all_tasks.end(); ++iter) {
        for (const TaskNode* expected_downstream : (*iter)->downstreams) {
          for (const TaskNode* candidate_downstream : downstream_tasks) {
            if (expected_downstream->id == candidate_downstream->id) {
              names.emplace((*iter)->output_class_name);
            }
          }
        }
      }
      return names;
    }

    TaskNodeConstPtrSet getTasks(const bool& inclusive = true) const {
      TaskNodeConstPtrSet all_tasks;
      if (inclusive) {
        for (auto iter = tasks.begin(); iter != tasks.end(); ++iter) {
          all_tasks.emplace(iter->second);
        }
      }
      for (auto iter = subgraphs.begin(); iter != subgraphs.end(); ++iter) {
        TaskNodeConstPtrSet subgraph_all_tasks = iter->second->getTasks(true);
        all_tasks.insert(subgraph_all_tasks.begin(), subgraph_all_tasks.end());
      }
      return all_tasks;
    }

    TaskNodeConstPtrSet getDownstreamTasks(const bool& inclusive = true) const {
      TaskNodeConstPtrSet downstream_tasks;
      if (inclusive) {
        for (auto iter = tasks.begin(); iter != tasks.end(); ++iter) {
          const TaskNode* task = iter->second;
          for (const TaskNode* downstream : task->downstreams) {
            downstream_tasks.emplace(downstream);
          }
        }
      }
      for (auto iter = subgraphs.begin(); iter != subgraphs.end(); ++iter) {
        TaskNodeConstPtrSet subgraph_downstream_tasks = iter->second->getDownstreamTasks(true);
        downstream_tasks.insert(subgraph_downstream_tasks.begin(), subgraph_downstream_tasks.end());
      }
      return downstream_tasks;
    }
  };

  using GraphConstPtrSet = std::unordered_set<const Graph*, IdHasher<Graph>, IdEquals<Graph>>;

  enum ArchitectureType {
    DEPLOYMENT = 0,
    ARCH_LVL_1,
    ARCH_LVL_2,
    ARCH_LVL_3,
  };

  std::string dot(const Workflow& workflow, const ArchitectureType& type) {
    std::stringstream ss;
    const std::unordered_map<void*, TaskNode>& tasks = workflow.getTasks();
    ss << "digraph G {" << std::endl;
    ss << "node [margin=0 fontcolor=blue fontsize=16 width=0.5 shape=ellipse style=filled]"
       << std::endl;
    ss << "URL=\"https://pub-gitlab.t3caic.com/caic-ad/incubator/stoicheia/-/blob/master/src/app/"
          "smartcar.cc\""
       << std::endl;
    // PRQA S 4400 ++ # TODO: to be solved
    switch (type) {
      case ARCH_LVL_1:
        ss << renderGraph(buildArchGraph(tasks), 1);
        break;
      case ARCH_LVL_2:
        ss << renderGraph(buildArchGraph(tasks), 2);
        break;
      case ARCH_LVL_3:
        ss << renderGraph(buildArchGraph(tasks), 3);
        break;
      case DEPLOYMENT:
        ss << renderGraph(buildDeploymentGraph(tasks), 3);
        break;
        // PRQA S 4400 --
    }
    ss << "}" << std::endl;
    return ss.str();
  }

 private:
  Graph buildArchGraph(const std::unordered_map<void*, TaskNode>& tasks) {  // PRQA S 6006
    Graph graph("Logic Architecture");
    for (auto iter = tasks.begin(); iter != tasks.end(); ++iter) {
      const TaskNode& task = iter->second;
      if (task.module == "") {
        graph.tasks.emplace(task.id, &task);
        continue;
      }
      auto module_iter = graph.subgraphs.find(task.module);
      if (module_iter == graph.subgraphs.end()) {
        module_iter =
            graph.subgraphs.emplace(task.module, std::make_shared<Graph>(task.module)).first;
      }
      Graph& module_graph = *module_iter->second;
      if (task.submodule == "") {
        module_graph.tasks.emplace(task.id, &task);
        continue;
      }
      auto submodule_iter = module_graph.subgraphs.find(task.submodule);
      if (submodule_iter == module_graph.subgraphs.end()) {
        submodule_iter =
            module_graph.subgraphs.emplace(task.submodule, std::make_shared<Graph>(task.submodule))
                .first;
      }
      Graph& submodule_graph = *submodule_iter->second;
      submodule_graph.tasks.emplace(task.id, &task);
    }
    return graph;
  }

  Graph buildDeploymentGraph(const std::unordered_map<void*, TaskNode>& tasks) {
    Graph graph("Physical Architecture");
    for (auto iter = tasks.begin(); iter != tasks.end(); ++iter) {
      const TaskNode& task = iter->second;
      if (task.module == "") {
        graph.tasks.emplace(task.id, &task);
        continue;
      }
      auto deploy_iter = graph.subgraphs.find(task.deployment);
      if (deploy_iter == graph.subgraphs.end()) {
        deploy_iter =
            graph.subgraphs.emplace(task.deployment, std::make_shared<Graph>(task.deployment))
                .first;
      }
      Graph& deploy_graph = *deploy_iter->second;
      deploy_graph.tasks.emplace(task.id, &task);
    }
    return graph;
  }

  std::string renderGraph(const Graph& graph, const int32_t& depth) {
    std::stringstream ss;
    ss << renderSubgraph(graph, depth);
    ss << renderEdges(graph, depth);
    return ss.str();
  }

  std::string renderSubgraph(const Graph& graph, const int32_t& depth) {
    std::stringstream ss;
    ss << "subgraph cluster_" << reinterpret_cast<int64_t>(graph.id) << " {" << std::endl;
    ss << "label=\"" << graph.name << "\"" << std::endl;
    ss << "fontsize=24" << std::endl;
    ss << "{" << std::endl;
    for (auto iter = graph.tasks.begin(); iter != graph.tasks.end(); ++iter) {
      ss << renderTask(*iter->second);
    }
    ss << "}" << std::endl;
    for (auto iter = graph.subgraphs.begin(); iter != graph.subgraphs.end(); ++iter) {
      if (depth == 1) {
        ss << renderGraphAsTask(*iter->second);
      } else {
        ss << renderSubgraph(*iter->second, depth - 1);
      }
    }
    ss << "}" << std::endl;
    return ss.str();
  }

  std::string renderInputs(const std::unordered_set<std::string>& names) {
    std::stringstream ss;
    ss << "Inputs:" << std::endl;
    for (const std::string& name : names) {
      ss << "  " << name << std::endl;
    }
    return ss.str();
  }

  std::string renderOutputs(const std::unordered_set<std::string>& names) {
    std::stringstream ss;
    ss << "Output:" << std::endl;
    for (const std::string& name : names) {
      ss << "  " << name << std::endl;
    }
    return ss.str();
  }

  std::string renderGraphAsTask(const Graph& graph) {
    std::stringstream ss;
    static std::string url =
        "https://pub-gitlab.t3caic.com/caic-ad/incubator/stoicheia/-/tree/master/src/";
    ss << reinterpret_cast<int64_t>(graph.id) << "[";
    ss << "label=\"" << graph.name << "\" ";
    ss << "color=\"" << graph.getColor() << "\" ";
    ss << "URL=\"" << url << "\" ";
    ss << "tooltip=\"" << renderInputs(graph.getInputClassNames())
       << renderOutputs(graph.getOutputClassNames()) << "\" ";
    ss << "]" << std::endl;
    return ss.str();
  }

  std::string renderTask(const TaskNode& task) {
    std::stringstream ss;
    static std::string url =
        "https://pub-gitlab.t3caic.com/caic-ad/incubator/stoicheia/-/tree/master/src/";
    ss << reinterpret_cast<int64_t>(task.id) << "[";
    ss << "label=\"" << task.display_name << "\" ";
    ss << "color=\"" << task.color << "\" ";
    ss << "URL=\"" << url << task.source_file << "\" ";
    ss << "tooltip=\"" << renderInputs(task.input_class_names)
       << renderOutputs({task.output_class_name}) << "\" ";
    ss << "]" << std::endl;
    return ss.str();
  }

  TaskNodeConstPtrSet getTaskNodes(const Graph& graph, const int32_t& depth) {
    TaskNodeConstPtrSet nodes;
    for (auto iter = graph.tasks.begin(); iter != graph.tasks.end(); ++iter) {
      nodes.emplace(iter->second);
    }
    if (depth > 1) {
      for (auto iter = graph.subgraphs.begin(); iter != graph.subgraphs.end(); ++iter) {
        TaskNodeConstPtrSet subgraph_nodes = getTaskNodes(*iter->second, depth - 1);
        nodes.insert(subgraph_nodes.begin(), subgraph_nodes.end());
      }
    }
    return nodes;
  }

  GraphConstPtrSet getLeafGraphs(const Graph& graph, const int32_t& depth) {
    GraphConstPtrSet graphs;
    for (auto iter = graph.subgraphs.begin(); iter != graph.subgraphs.end(); ++iter) {
      if (depth == 1) {
        graphs.emplace(iter->second.get());
      } else {
        GraphConstPtrSet subgraphs = getLeafGraphs(*iter->second, depth - 1);
        graphs.insert(subgraphs.begin(), subgraphs.end());
      }
    }
    return graphs;
  }

  std::string renderEdges(const Graph& graph, const int32_t& depth) {
    std::stringstream ss;
    GraphConstPtrSet leaves = getLeafGraphs(graph, depth);
    TaskNodeConstPtrMap<const Graph*> task_2_graphs;
    for (const Graph* leaf : leaves) {
      TaskNodeConstPtrSet tasks = leaf->getTasks();
      for (const TaskNode* task : tasks) {
        task_2_graphs.emplace(task, leaf);
      }
    }
    PairVoidConstPtrSet edges;
    for (const Graph* leaf : leaves) {
      TaskNodeConstPtrSet tasks = leaf->getDownstreamTasks();
      for (const TaskNode* task : tasks) {
        auto iter = task_2_graphs.find(task);
        if (iter == task_2_graphs.end()) {
          ss << renderEdge(leaf->id, task->id, leaf->getOutputClassNames({task}), &edges,
                           leaf->getColor());
        } else {
          ss << renderEdge(leaf->id, iter->second->id,
                           leaf->getOutputClassNames(iter->second->getTasks()), &edges,
                           leaf->getColor());
        }
      }
    }
    TaskNodeConstPtrSet tasks = getTaskNodes(graph, depth);
    for (const TaskNode* task : tasks) {
      for (const TaskNode* downstream : task->downstreams) {
        auto iter = task_2_graphs.find(downstream);
        if (iter == task_2_graphs.end()) {
          ss << renderEdge(task->id, downstream->id, {task->output_class_name}, &edges,
                           task->color);
        } else {
          ss << renderEdge(task->id, iter->second->id, {task->output_class_name}, &edges,
                           task->color);
        }
      }
    }
    return ss.str();
  }

  std::string renderEdge(const void* from, const void* to,
                         const std::unordered_set<std::string>& output_class_names,
                         PairVoidConstPtrSet* const edges, const std::string& edge_color) {
    if (from == to) {
      return "";
    }
    bool exist = !edges->emplace(std::make_pair(from, to)).second;
    if (exist) {
      return "";
    }
    std::stringstream ss;
    ss << reinterpret_cast<int64_t>(from) << " -> " << reinterpret_cast<int64_t>(to) << std::endl;
    ss << "[";
    ss << "color=\"" << edge_color << "\" ";
    ss << "arrowhead=\"onormal\" ";
    ss << "tooltip=\"" << renderOutputs(output_class_names) << "\" ";
    ss << "penwidth=2.0 ";
    ss << "]" << std::endl;
    return ss.str();
  }
};

}  // namespace cm::pattern
