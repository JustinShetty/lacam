#include <lacam.hpp>
#include <utils.hpp>

#include "gtest/gtest.h"

static bool VERBOSITY = 5;

// TEST(planner, basic)
// {
//   const auto map_filename = "./tests/assets/2x2.map";
//   const auto N = 2;
//   const std::vector<int> starts = {0, 3};
//   const std::vector<int> goals = {3, 0};
//   const Instance ins(map_filename, starts, goals);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   auto solution = solve(ins, VERBOSITY, nullptr, nullptr, N);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, N, false));
// }

// TEST(planner, basic_with_orientation)
// {
//   const auto map_filename = "./tests/assets/2x2.map";
//   const auto G = std::shared_ptr<Graph>(new Graph(map_filename));
//   const auto N = 2;
//   const std::vector<State> starts = {
//       State(G->U[0], 0, Orientation::X_MINUS),
//       State(G->U[3], 0, Orientation::X_PLUS),
//   };
//   const std::vector<std::vector<State>> goals{
//       {State(G->U[3], 0, Orientation::Y_PLUS)},
//       {State(G->U[0], 0, Orientation::Y_MINUS)},
//   };
//   const Instance ins(G, starts, goals);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   auto solution = solve(ins, VERBOSITY, nullptr, nullptr, N);
//   ASSERT_EQ(solution.size(), 6);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, N, false));
// }

// TEST(planner, solve)
// {
//   const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
//   const auto map_filename = "./assets/random-32-32-10.map";
//   const auto N = 3;
//   const auto ins = Instance(scen_filename, map_filename, N);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   auto solution = solve(ins, VERBOSITY, nullptr, nullptr, N);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, N, false));
// }

// TEST(planner, already_solved)
// {
//   const auto map_filename = "./tests/assets/2x2.map";
//   const auto N = 2;
//   const std::vector<int> starts = {0, 3};
//   const std::vector<int> goals = {0, 3};
//   const Instance ins(map_filename, starts, goals);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   auto solution = solve(ins, VERBOSITY, nullptr, nullptr, N);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, N, false));
// }

// TEST(planner, benchmark)
// {
//   auto MT = std::mt19937(0);
//   const auto allow_following = true;
//   const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
//   const auto map_filename = "./assets/random-32-32-10.map";
//   const auto N = 50;

//   const auto ins = Instance(scen_filename, map_filename, N);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   auto solution = solve(ins, VERBOSITY, nullptr, &MT, N, allow_following);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(
//       is_feasible_solution(ins, solution, VERBOSITY, N, allow_following));
// }

TEST(planner, benchmark_with_orientation)
{
  auto MT = std::mt19937(0);
  const auto allow_following = true;
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto N = 50;

  auto ins = Instance(scen_filename, map_filename, N);
  ASSERT_TRUE(ins.is_valid(VERBOSITY));
  std::uniform_int_distribution<int> dist(1, 4); // uniform over [1, 4]
  for (int i = 0; i < N; i++)
  {
    ins.starts[i].o = static_cast<Orientation::Value>(dist(MT));
    ins.goal_sequences[i][0].o = static_cast<Orientation::Value>(dist(MT));
  }
  ASSERT_TRUE(ins.is_valid(VERBOSITY));

  auto solution = solve(ins, VERBOSITY, nullptr, &MT, N, allow_following);
  ASSERT_GT(solution.size(), 0);
  ASSERT_TRUE(
      is_feasible_solution(ins, solution, VERBOSITY, N, allow_following));
}

// TEST(planner, solve_multiple_goals_2x2)
// {
//   const auto map_filename = "./tests/assets/2x2.map";

//   const std::vector<int> starts = {0};
//   std::vector<std::vector<int>> goal_sequences = {{3, 0}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = ins.get_total_goals();
//   auto solution = solve(ins, VERBOSITY, nullptr, nullptr, threshold);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold, false));
// }

// TEST(planner, solve_multiple_goals_2x2_2agents)
// {
//   const auto map_filename = "./tests/assets/2x2.map";

//   const std::vector<int> starts = {0, 3};
//   std::vector<std::vector<int>> goal_sequences = {{3, 0}, {0, 3}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = ins.get_total_goals();
//   auto solution = solve(ins, VERBOSITY, nullptr, nullptr, threshold);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold, false));
// }

// TEST(planner, solve_multiple_goals_32x32a)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";
//   const std::vector<int> starts = {0};
//   std::vector<std::vector<int>> goal_sequences = {
//       {1023, 1021, 1023, 1021, 1023, 1021, 1023, 1021, 1023, 1021, 1023,
//        1021, 1023, 1021, 1023, 1021, 1023, 1021, 1023, 1021, 1023, 0}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = ins.get_total_goals();
//   const bool allow_following = false;
//   auto solution =
//       solve(ins, VERBOSITY, nullptr, nullptr, threshold, allow_following);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold,
//                                    allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32b)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";
//   const std::vector<int> starts = {0};
//   std::vector<std::vector<int>> goal_sequences = {
//       {1023, 1021, 1023, 1021, 1023, 1021, 1023, 1021, 1023, 1021, 1023, 1021,
//        1023, 1021, 1023, 1021, 1023, 1021, 1023, 1021, 1023, 0,    1023}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = ins.get_total_goals();
//   const bool allow_following = false;
//   auto solution =
//       solve(ins, VERBOSITY, nullptr, nullptr, threshold, allow_following);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold,
//                                    allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32_2agents0)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";

//   const std::vector<int> starts = {174, 662};
//   std::vector<std::vector<int>> goal_sequences = {{0}, {992}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = ins.get_total_goals();
//   const bool allow_following = false;
//   auto solution =
//       solve(ins, VERBOSITY, nullptr, nullptr, threshold, allow_following);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold,
//                                    allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32_2agents1)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";

//   const std::vector<int> starts = {174, 662};
//   std::vector<std::vector<int>> goal_sequences = {{0, 1023}, {992, 31}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = ins.get_total_goals();
//   const bool allow_following = false;
//   auto solution =
//       solve(ins, VERBOSITY, nullptr, nullptr, threshold, allow_following);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold,
//                                    allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32_3agents0)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";

//   const std::vector<int> starts = {174, 662, 0};
//   std::vector<std::vector<int>> goal_sequences = {{0}, {992}, {1023}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = ins.get_total_goals();
//   const bool allow_following = false;
//   auto solution =
//       solve(ins, VERBOSITY, nullptr, nullptr, threshold, allow_following);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold,
//                                    allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32_3agents1)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";

//   const std::vector<int> starts = {174, 662, 0};
//   std::vector<std::vector<int>> goal_sequences = {{0, 1022}, {992}, {1023}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = ins.get_total_goals();
//   const bool allow_following = false;
//   auto solution =
//       solve(ins, VERBOSITY, nullptr, nullptr, threshold, allow_following);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold,
//                                    allow_following));
// }

// TEST(planner, solve_multiple_goals_32x32_3agents2)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";

//   const std::vector<int> starts = {174, 662, 0};
//   std::vector<std::vector<int>> goal_sequences = {
//       {0, 1023}, {992, 31}, {1023, 662}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = ins.get_total_goals();
//   const bool allow_following = false;
//   auto solution =
//       solve(ins, VERBOSITY, nullptr, nullptr, threshold, allow_following);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold,
//                                    allow_following));
// }

// TEST(planner, solve_multiple_goals_thresholded)
// {
//   const auto map_filename = "./tests/assets/2x2.map";

//   const std::vector<int> starts = {0};
//   std::vector<std::vector<int>> goal_sequences = {{3, 0}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = 1;
//   auto solution = solve(ins, VERBOSITY, nullptr, nullptr, threshold);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold, false));
// }

// TEST(planner, solver_multiple_goals_thresholded2)
// {
//   const auto map_filename = "./assets/random-32-32-10.map";

//   const std::vector<int> starts = {0, 1, 2};
//   std::vector<std::vector<int>> goal_sequences = {{1}, {0}, {3, 30, 60}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = 3;
//   const bool allow_following = false;
//   auto solution =
//       solve(ins, VERBOSITY, nullptr, nullptr, threshold, allow_following);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold,
//                                    allow_following));
// }

// TEST(planner, solver_duplicate_goal_end)
// {
//   const auto map_filename = "./tests/assets/2x2.map";

//   const std::vector<int> starts = {0, 1};
//   std::vector<std::vector<int>> goal_sequences = {{2}, {2}};
//   const auto ins = Instance(map_filename, starts, goal_sequences);
//   ASSERT_TRUE(ins.is_valid(VERBOSITY));

//   const auto threshold = ins.get_total_goals();
//   const bool allow_following = false;
//   auto solution =
//       solve(ins, VERBOSITY, nullptr, nullptr, threshold, allow_following);
//   ASSERT_GT(solution.size(), 0);
//   ASSERT_TRUE(is_feasible_solution(ins, solution, VERBOSITY, threshold,
//                                    allow_following));
// }
