/*
 * distance table with lazy evaluation, using BFS
 */
#pragma once

#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

struct DistTableMultiGoal {
  const Instance* ins;
  std::vector<std::unordered_map<StatePtr, int, StatePtrHasher>>
      table;  // distance table, index: agent-id, State
  std::vector<std::queue<StatePtr>> OPEN;  // search queues

  int get(int agent_id, const StatePtr from);

  DistTableMultiGoal(const Instance* _ins);
  DistTableMultiGoal(const Instance& _ins) : DistTableMultiGoal(&_ins) {}

  void setup(const Instance* ins);  // initialization
};
