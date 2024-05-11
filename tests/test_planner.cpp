#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(planner, solve)
{
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto N = 3;
  const auto ins = Instance(scen_filename, map_filename, N);

  auto solution = solve(ins, 0, nullptr, nullptr, N);
  ASSERT_TRUE(is_feasible_solution(ins, solution));
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
