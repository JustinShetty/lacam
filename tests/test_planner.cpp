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
  ASSERT_GT(solution.size(), 0);
  ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, false));
}

TEST(planner, already_solved)
{
  const auto map_filename = "./tests/assets/2x2.map";
  const auto N = 2;
  const std::vector<int> starts = {0, 3};
  const std::vector<int> goals = {0, 3};
  const Instance ins(map_filename, starts, goals);

  auto solution = solve(ins, 0, nullptr, nullptr, N);
  ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, false));
  ASSERT_EQ(solution.size(), 1);
}

TEST(planner, benchmark)
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

// TEST(planner, solve_multiple_goals_32x32)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";
//   const auto N = 1;

//   const std::vector<int> starts = {174};
//   std::vector<std::vector<int>> goal_sequences = {{0, 1023}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);

//   const bool allow_following = false;
//   auto solution = solve(ins, 0, nullptr, nullptr, N, allow_following);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32_2agents0)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";
//   const auto N = 2;

//   const std::vector<int> starts = {174, 662};
//   std::vector<std::vector<int>> goal_sequences = {{0}, {992}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);

//   const bool allow_following = false;
//   auto solution = solve(ins, 0, nullptr, nullptr, N, allow_following);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32_2agents1)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";
//   const auto N = 2;

//   const std::vector<int> starts = {174, 662};
//   std::vector<std::vector<int>> goal_sequences = {{0, 1023}, {992, 31}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);

//   const bool allow_following = false;
//   auto solution = solve(ins, 0, nullptr, nullptr, N, allow_following);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32_3agents0)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";
//   const auto N = 2;

//   const std::vector<int> starts = {174, 662, 0};
//   std::vector<std::vector<int>> goal_sequences = {{0}, {992}, {1023}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);

//   const bool allow_following = false;
//   auto solution = solve(ins, 0, nullptr, nullptr, N, allow_following);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32_3agents1)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";
//   const auto N = 2;

//   const std::vector<int> starts = {174, 662, 0};
//   std::vector<std::vector<int>> goal_sequences = {
//       {0, 1023}, {992}, {1023}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);

//   const bool allow_following = false;
//   auto solution = solve(ins, 0, nullptr, nullptr, N, allow_following);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32_3agents2)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";
//   const auto N = 2;

//   const std::vector<int> starts = {174, 662, 0};
//   std::vector<std::vector<int>> goal_sequences = {
//       {0, 1023}, {992, 31}, {1023}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);

//   const bool allow_following = false;
//   auto solution = solve(ins, 0, nullptr, nullptr, N, allow_following);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32_3agents2)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";
//   const auto N = 2;

//   const std::vector<int> starts = {174, 662, 0};
//   std::vector<std::vector<int>> goal_sequences = {
//       {0, 1023}, {992, 31}, {1023, 662}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);

//   const bool allow_following = false;
//   auto solution = solve(ins, 0, nullptr, nullptr, N, allow_following);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, 1, N, allow_following));
// }