#ifndef _GRAPH_SEARCH_H_
#define _GRAPH_SEARCH_H_

#include <vector>
#include <unordered_map>
#include <queue>
#include <list>
#include <memory>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <Eigen/Eigen>

using std::list;
using std::queue;
using std::priority_queue;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using std::cout;
using Eigen::Vector3d;

namespace fast_planner {
// GraphSearch that operates on different types of node using Dijkstra algorithm
template <typename NodeT>
class GraphSearch {
public:
  GraphSearch() {
    node_num_ = 0;
    edge_num_ = 0;
  }
  ~GraphSearch() {
  }

  void print();
  void addNode(const shared_ptr<NodeT>& node);
  void addEdge(const int& from, const int& to);
  void DijkstraSearch(const int& start, const int& goal, vector<shared_ptr<NodeT>>& path);

private:
  vector<shared_ptr<NodeT>> nodes_;
  int node_num_;
  int edge_num_;
};

template <typename NodeT>
class NodeCompare {
public:
  bool operator()(const shared_ptr<NodeT>& node1, const shared_ptr<NodeT>& node2) {
    return node1->g_value_ > node2->g_value_;
  }
};

template <typename NodeT>
void GraphSearch<NodeT>::print() {
  for (auto v : nodes_) {
    v->print();
    v->printNeighbors();
  }
}

template <typename NodeT>
void GraphSearch<NodeT>::addNode(const shared_ptr<NodeT>& node) {
  nodes_.push_back(node);
  nodes_.back()->id_ = node_num_++;
}

template <typename NodeT>
void GraphSearch<NodeT>::addEdge(const int& from, const int& to) {
  nodes_[from]->neighbors_.push_back(nodes_[to]);
  ++edge_num_;
}

template <typename NodeT>
void GraphSearch<NodeT>::DijkstraSearch(const int& start, const int& goal,
                                        vector<shared_ptr<NodeT>>& path) {
  std::cout << "Node: " << node_num_ << ", edge: " << edge_num_ << std::endl;
  // Basic structure used by Dijkstra
  // unordered_map<int, int> close_set;
  priority_queue<shared_ptr<NodeT>, vector<shared_ptr<NodeT>>, NodeCompare<NodeT>> open_set;

  shared_ptr<NodeT> start_v = nodes_[start];
  shared_ptr<NodeT> end_v = nodes_[goal];
  start_v->g_value_ = 0.0;
  open_set.push(start_v);

  while (!open_set.empty()) {
    auto vc = open_set.top();
    open_set.pop();
    vc->closed_ = true;
    // close_set[vc->id_] = 1;

    // Check if reach target
    if (vc == end_v) {
      // std::cout << "Dijkstra reach target" << std::endl;
      shared_ptr<NodeT> vit = vc;
      while (vit != nullptr) {
        path.push_back(vit);
        vit = vit->parent_;
      }
      reverse(path.begin(), path.end());
      return;
    }
    for (auto vb : vc->neighbors_) {
      // Check if in close set
      if (vb->closed_) continue;

      // Add new node or updated node in open set
      double g_tmp = vc->g_value_ + vc->costTo(vb);
      if (g_tmp < vb->g_value_) {
        vb->g_value_ = g_tmp;
        vb->parent_ = vc;
        open_set.push(vb);
      }
    }
  }
}
}

#endif