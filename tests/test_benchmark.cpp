#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(solve, benchmark)
{
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto N = 50;
  const auto ins = Instance(scen_filename, map_filename, N);
  const auto allow_following = true;
  const auto verbosity = 2;
  auto solution = solve(ins, verbosity, nullptr, nullptr, N, allow_following);
  ASSERT_TRUE(is_feasible_solution(ins, solution, verbosity, N, allow_following));
}
