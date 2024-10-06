#include <lacam.hpp>

#include "gtest/gtest.h"

static bool VERBOSITY = 0;

TEST(PostProcessing, validate_following)
{
  const auto map_filename = "./assets/empty-8-8.map";
  const auto N = 2;
  const auto allow_following = true;
  const auto start_indexes = std::vector<int>({0, 8});
  const auto goal_indexes = std::vector<int>({9, 1});
  const auto ins = Instance(map_filename, start_indexes, goal_indexes);

  // correct solution
  auto sol = Solution(3);
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[1], ins.G.U[0]});
  sol[2] = Config({ins.G.U[9], ins.G.U[1]}, {1, 1});
  ASSERT_TRUE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));

  // invalid start
  sol[0] = Config({ins.G.U[0], ins.G.U[4]});
  sol[1] = Config({ins.G.U[1], ins.G.U[0]});
  sol[2] = Config({ins.G.U[9], ins.G.U[1]}, {1, 1});
  ASSERT_FALSE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));

  // invalid goal
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[1], ins.G.U[0]});
  sol[2] = Config({ins.G.U[10], ins.G.U[1]}, {0, 1});
  ASSERT_FALSE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));

  // invalid transition
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[4], ins.G.U[0]});
  sol[2] = Config({ins.G.U[9], ins.G.U[1]}, {1, 1});
  ASSERT_FALSE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));

  // // swap conflict
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[8], ins.G.U[0]});
  sol[2] = Config({ins.G.U[9], ins.G.U[1]}, {1, 1});
  ASSERT_FALSE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));

  // vertex conflict
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[0], ins.G.U[0]});
  sol[2] = Config({ins.G.U[8], ins.G.U[1]});
  sol.push_back(Config({ins.G.U[9], ins.G.U[1]}, {1, 1}));
  ASSERT_FALSE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));
}

TEST(PostProcessing, validate_no_following)
{
  const auto map_filename = "./assets/empty-8-8.map";
  const auto N = 2;
  const auto allow_following = false;
  const auto start_indexes = std::vector<int>({0, 8});
  const auto goal_indexes = std::vector<int>({9, 1});
  const auto ins = Instance(map_filename, start_indexes, goal_indexes);

  // correct solution
  auto sol = Solution(4);
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[1], ins.G.U[8]});
  sol[2] = Config({ins.G.U[9], ins.G.U[0]}, {1, 0});
  sol[3] = Config({ins.G.U[9], ins.G.U[1]}, {1, 1});
  ASSERT_TRUE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));

  // invalid start
  sol[0] = Config({ins.G.U[0], ins.G.U[4]});
  sol[1] = Config({ins.G.U[1], ins.G.U[8]});
  sol[2] = Config({ins.G.U[9], ins.G.U[0]}, {1, 0});
  sol[3] = Config({ins.G.U[9], ins.G.U[1]}, {1, 1});
  ASSERT_FALSE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));

  // invalid goal
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[1], ins.G.U[8]});
  sol[2] = Config({ins.G.U[5], ins.G.U[0]}, {0, 0});
  sol[3] = Config({ins.G.U[10], ins.G.U[1]}, {0, 1});
  ASSERT_FALSE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));

  // invalid transition
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[10], ins.G.U[8]});
  sol[2] = Config({ins.G.U[9], ins.G.U[0]}, {1, 0});
  sol[3] = Config({ins.G.U[9], ins.G.U[1]}, {1, 1});
  ASSERT_FALSE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));

  // following conflict
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[1], ins.G.U[0]});
  sol[2] = Config({ins.G.U[9], ins.G.U[1]}, {1, 1});
  sol[3] = Config({ins.G.U[9], ins.G.U[1]}, {1, 1});
  ASSERT_FALSE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));

  // vertex conflict
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[1], ins.G.U[8]});
  sol[2] = Config({ins.G.U[9], ins.G.U[0]}, {1, 0});
  sol[3] = Config({ins.G.U[1], ins.G.U[1]}, {1, 1});
  ASSERT_FALSE(is_feasible_solution(ins, sol, VERBOSITY, N, allow_following));
}

TEST(PostProcessing, metrics)
{
  const auto N = 3;
  const auto map_filename = "./assets/empty-8-8.map";
  const auto start_indexes = std::vector<int>({0, 5, 10});
  const auto goal_indexes = std::vector<int>({2, 4, 11});
  const auto ins = Instance(map_filename, start_indexes, goal_indexes);

  // correct solution
  auto sol = Solution(3);
  sol[0] = Config({ins.G.U[0], ins.G.U[5], ins.G.U[10]});
  sol[1] = Config({ins.G.U[1], ins.G.U[4], ins.G.U[11]});
  sol[2] = Config({ins.G.U[2], ins.G.U[4], ins.G.U[11]}, {1, 1, 1});

  ASSERT_TRUE(is_feasible_solution(ins, sol, VERBOSITY, N, true));
  ASSERT_EQ(get_makespan(sol), 2);
  ASSERT_EQ(get_sum_of_costs(sol), 4);
}
