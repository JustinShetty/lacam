#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(PostProcesing, validate)
{
  const auto map_filename = "./assets/empty-8-8.map";
  const auto verbose = 0;
  const auto N = 2;
  const auto start_indexes = std::vector<int>({0, 8});
  const auto goal_indexes = std::vector<int>({9, 1});
  const auto ins = Instance(map_filename, start_indexes, goal_indexes);

  // correct solution
  auto sol = Solution(4);
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[1], ins.G.U[8]});
  sol[2] = Config({ins.G.U[9], ins.G.U[0]});
  sol[3] = Config({ins.G.U[9], ins.G.U[1]});
  ASSERT_TRUE(is_feasible_solution(ins, sol, verbose, N));

  // invalid start
  sol[0] = Config({ins.G.U[0], ins.G.U[4]});
  sol[1] = Config({ins.G.U[1], ins.G.U[8]});
  sol[2] = Config({ins.G.U[9], ins.G.U[0]});
  sol[3] = Config({ins.G.U[9], ins.G.U[1]});
  ASSERT_FALSE(is_feasible_solution(ins, sol, verbose, N));

  // invalid goal
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[1], ins.G.U[8]});
  sol[2] = Config({ins.G.U[9], ins.G.U[0]});
  sol[3] = Config({ins.G.U[10], ins.G.U[1]});
  ASSERT_FALSE(is_feasible_solution(ins, sol, verbose, N));

  // invalid transition
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[10], ins.G.U[8]});
  sol[2] = Config({ins.G.U[9], ins.G.U[0]});
  sol[3] = Config({ins.G.U[9], ins.G.U[1]});
  ASSERT_FALSE(is_feasible_solution(ins, sol, verbose, N));

  // following conflict
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[1], ins.G.U[0]});
  sol[2] = Config({ins.G.U[9], ins.G.U[1]});
  sol[3] = Config({ins.G.U[9], ins.G.U[1]});
  ASSERT_FALSE(is_feasible_solution(ins, sol, verbose, N));

  // vertex conflict
  sol[0] = Config({ins.G.U[0], ins.G.U[8]});
  sol[1] = Config({ins.G.U[1], ins.G.U[8]});
  sol[2] = Config({ins.G.U[9], ins.G.U[0]});
  sol[3] = Config({ins.G.U[1], ins.G.U[1]});
  ASSERT_FALSE(is_feasible_solution(ins, sol, verbose, N));
}

TEST(PostProcessing, metrics)
{
  const auto map_filename = "./assets/empty-8-8.map";
  const auto start_indexes = std::vector<int>({0, 5, 10});
  const auto goal_indexes = std::vector<int>({2, 4, 11});
  const auto ins = Instance(map_filename, start_indexes, goal_indexes);

  // correct solution
  auto sol = Solution(3);
  sol[0] = Config({ins.G.U[0], ins.G.U[5], ins.G.U[10]});
  sol[1] = Config({ins.G.U[1], ins.G.U[4], ins.G.U[11]});
  sol[2] = Config({ins.G.U[2], ins.G.U[4], ins.G.U[11]});

  ASSERT_EQ(get_makespan(sol), 2);
  ASSERT_EQ(get_sum_of_costs(sol), 4);
}
