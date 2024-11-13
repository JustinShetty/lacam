/*
 * LaCAM algorithm
 */
#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"
#include <optional>

// low-level search node
struct Constraint {
  std::vector<int> who;
  std::vector<State> where;
  const int depth;
  Constraint();
  Constraint(Constraint* parent, int i, State s);  // who and where
  ~Constraint();
};

// high-level search node
struct Node {
  const Config C;
  Node* parent;

  // for low-level search
  std::vector<float> priorities;
  std::vector<int> order;
  std::queue<Constraint*> search_tree;

  Node(Config _C, DistTableMultiGoal& D, Node* _parent = nullptr);
  ~Node();
};
using Nodes = std::vector<Node*>;

// PIBT agent
struct Agent {
  const int id;
  State s_now;   // current state
  State s_next;  // next state
  Agent(int _id) : id(_id), s_now(), s_next() {}
};
using Agents = std::vector<Agent*>;

struct Planner {
  const Instance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;
  const std::optional<int> threshold;
  const bool allow_following;

  // solver utils
  const int N;  // number of agents
  const int V_size;
  DistTableMultiGoal D;
  std::vector<float> tie_breakers;  // random values, used in PIBT
  Agents A;
  Agents occupied_now;   // for quick collision checking
  Agents occupied_next;  // for quick collision checking

  Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          int _verbose = 0, const std::optional<int> threshold = std::nullopt, bool _allow_following = false);
  Solution solve();
  bool get_new_config(Node* S, Constraint* M);
  bool funcPIBT(Agent* ai, Agent* caller = nullptr);
};

// main function
Solution solve(const Instance& ins, const int verbose = 0,
               const Deadline* deadline = nullptr, std::mt19937* MT = nullptr,
               const std::optional<int> threshold = std::nullopt, const bool allow_following = false);
