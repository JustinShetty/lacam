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
  const auto G = std::make_shared<Graph>(map_filename);
  const std::vector<StatePtr> starts = {
      G->NewState(G->U[0], 0, Orientation::X_MINUS)};  // (0,0)
  const std::vector<std::vector<StatePtr>> goal_sequences = {
      {G->NewState(G->U[3], 0, Orientation::Y_PLUS)}};  // (1,1)
  const auto ins = Instance(G, starts, goal_sequences);
  auto dist_table = DistTableMultiGoal(ins);

  ASSERT_EQ(dist_table.get(0, ins.goal_sequences[0].back()), 0);
  ASSERT_EQ(dist_table.get(0, ins.starts[0]), 5);
}

TEST(dist_table, multiple)
{
  const auto map_filename = "./tests/assets/2x2.map";
  const auto G = std::make_shared<Graph>(map_filename);
  const std::vector<StatePtr> starts = {G->NewState(G->U[0], 0)};  // (0,0)
  const std::vector<std::vector<StatePtr>> goal_sequences = {
      {
          G->NewState(G->U[3], 0),  // (1,1)
          G->NewState(G->U[0], 1),  // (0,0)
      },
  };
  const auto ins = Instance(G, starts, goal_sequences);
  auto dist_table = DistTableMultiGoal(ins);

  ASSERT_EQ(dist_table.get(0, ins.goal_sequences[0][0]), 0);
  ASSERT_EQ(dist_table.get(0, ins.goal_sequences[0][1]), 0);
}
