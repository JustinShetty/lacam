/*
 * distance table with lazy evaluation, using BFS
 */
#pragma once

#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

struct DistTableMultiGoal {
  const int K;  // number of vertices
  std::vector<std::vector<std::vector<int>>>
      table;  // distance table, index: agent-id, goal_index, vertex-id
  std::vector<std::vector<std::queue<Vertex*>>> OPEN;  // search queues

  int get(int agent_id, int goal_index, int from_id);
  inline int get(int agent_id, int goal_index, Vertex* from)
  {
    return get(agent_id, goal_index, from->id);
  }

  DistTableMultiGoal(const Instance* ins);
  DistTableMultiGoal(const Instance& ins) : DistTableMultiGoal(&ins) {}

  void setup(const Instance* ins);  // initialization
};
