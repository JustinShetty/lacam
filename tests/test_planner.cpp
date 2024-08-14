#include <lacam.hpp>
#include <utils.hpp>

#include "gtest/gtest.h"

TEST(planner, solve)
{
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto N = 3;
  const auto ins = Instance(scen_filename, map_filename, N);

  auto solution = solve(ins, 0, nullptr, nullptr, N);
  ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, false));
}

TEST(planner, unsolvable_instance)
{
  const auto scen_filename = "./tests/assets/2x1.scen";
  const auto map_filename = "./tests/assets/2x1.map";
  const auto N = 2;
  const auto ins = Instance(scen_filename, map_filename, N);

  auto solution = solve(ins, 0, nullptr, nullptr, N);
  ASSERT_TRUE(solution.empty());
}

TEST(planner, solve_multiple_goals_2x2)
{
  const auto map_filename = "./tests/assets/2x2.map";
  const auto N = 1;

  const std::vector<int> starts = {0};
  std::vector<std::vector<int>> goal_sequences = {{1, 0, 3, 2, 0}};
  const auto ins = Instance(map_filename, starts, goal_sequences);

  auto solution = solve(ins, 0, nullptr, nullptr, N);

  ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, false));
}

TEST(planner, solve_multiple_goals_2x2_2agents)
{
  const auto map_filename = "./tests/assets/2x2.map";
  const auto N = 2;

  const std::vector<int> starts = {0, 3};
  std::vector<std::vector<int>> goal_sequences = {{3, 0}, {0, 3}};
  const auto ins = Instance(map_filename, starts, goal_sequences);

  auto solution = solve(ins, 0, nullptr, nullptr, N);

  ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, false));
}