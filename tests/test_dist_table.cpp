#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(dist_table, init)
{
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto ins = Instance(scen_filename, map_filename, 3);
  auto dist_table = DistTableMultiGoal(ins);

  ASSERT_EQ(dist_table.get(0, ins.goal_sequences[0].back()), 0);
  ASSERT_EQ(dist_table.get(0, ins.starts[0]), 16);
}

TEST(dist_table, orientation)
{
  const auto map_filename = "./tests/assets/2x2.map";
  const auto G = Graph(map_filename);
  const std::vector<State> starts = {
      State(G.U[0], Orientation::DOWN, 0)};  // (0,0)
  const std::vector<std::vector<State>> goal_sequences = {
      {State(G.U[3], Orientation::UP, 0)}};  // (1,1)
  const auto ins = Instance(std::move(G), starts, goal_sequences);
  auto dist_table = DistTableMultiGoal(ins);

  ASSERT_EQ(dist_table.get(0, ins.goal_sequences[0].back()), 0);
  ASSERT_EQ(dist_table.get(0, ins.starts[0]), 4);
}
