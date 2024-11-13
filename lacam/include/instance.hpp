/*
 * instance definition
 */
#pragma once
#include <memory>
#include <random>

#include "graph.hpp"
#include "utils.hpp"

struct Instance {
  const std::shared_ptr<Graph> G;                  // graph
  Config starts;                                   // initial configuration
  std::vector<std::vector<State>> goal_sequences;  // agent id -> goal sequence
  const uint N;                                    // number of agents

  // for testing
  Instance(const std::string& map_filename,
           const std::vector<int>& start_indexes,
           const std::vector<int>& goal_indexes);
  Instance(const std::string& map_filename,
           const std::vector<int>& start_indexes,
           const std::vector<std::vector<int>>& goal_index_sequences);
  Instance(const std::shared_ptr<Graph> _G, const std::vector<State>& _starts,
           const std::vector<std::vector<State>>& _goal_sequences);
  // for MAPF benchmark
  Instance(const std::string& scen_filename, const std::string& map_filename,
           const int _N = 1);
  // random instance generation
  Instance(const std::string& map_filename, std::mt19937* MT, const int _N = 1);
  ~Instance() {}

  // simple feasibility check of instance
  bool is_valid(const int verbose = 0) const;

  int get_total_goals() const;

  bool is_goal_config(const Config& C) const;

  void update_goal_indices(Config& c,
                                          const Config& c_prev) const;
};

// solution: a sequence of configurations
using Solution = std::vector<Config>;
