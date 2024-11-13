/*
 * post processing, e.g., calculating solution quality
 */
#pragma once
#include <optional>

#include "dist_table.hpp"
#include "instance.hpp"
#include "utils.hpp"

bool is_feasible_solution(const Instance& ins, const Solution& solution,
                          const int verbose, const std::optional<int> threshold,
                          const bool allow_following);
int get_makespan(const Solution& solution);
int get_path_cost(const Solution& solution, int i);  // single-agent path cost
int get_sum_of_costs(const Solution& solution);
int get_sum_of_loss(const Solution& solution);
int get_makespan_lower_bound(const Instance& ins, DistTableMultiGoal& D);
int get_sum_of_costs_lower_bound(const Instance& ins, DistTableMultiGoal& D);
void print_stats(const int verbose, const Instance& ins,
                 const Solution& solution, const double comp_time_ms);
void make_log(const Instance& ins, const Solution& solution,
              const std::string& output_name, const double comp_time_ms,
              const std::string& map_name, const int seed,
              const bool log_short = false,  // true -> paths not appear
              const bool skip_post_processing = false);
