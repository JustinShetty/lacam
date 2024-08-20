#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(solve, benchmark)
{
  auto MT = std::mt19937(0);
  const auto allow_following = true;
  const auto verbosity = 0;
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto N = 50;

  const auto ins = Instance(scen_filename, map_filename, N);
  ASSERT_TRUE(ins.is_valid(verbosity));

  auto solution = solve(ins, verbosity, nullptr, &MT, N, allow_following);
  ASSERT_TRUE(
      is_feasible_solution(ins, solution, verbosity, N, allow_following));
}
