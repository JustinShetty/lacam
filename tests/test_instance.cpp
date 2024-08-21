#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(Instance, initialize)
{
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto ins = Instance(scen_filename, map_filename, 3);

  ASSERT_EQ(size(ins.starts), 3);
  ASSERT_EQ(size(ins.goals), 3);
  ASSERT_EQ(ins.starts[0]->index, 203);
  ASSERT_EQ(ins.goals[0]->index, 583);

  ASSERT_EQ(ins.goals.goal_indices, std::vector<int>({1, 1, 1}));
}

TEST(Instance, sequence)
{
  const auto map_filename = "./assets/empty-8-8.map";
  const auto ins = Instance(map_filename, {0, 1}, {{2, 3, 4}, {5, 6, 7}});

  ASSERT_EQ(size(ins.starts), 2);
  ASSERT_EQ(size(ins.goals), 2);
  ASSERT_EQ(ins.starts[0]->index, 0);
  ASSERT_EQ(ins.goals[0]->index, 4);

  ASSERT_EQ(ins.goal_sequences.size(), 2);
  ASSERT_EQ(ins.goal_sequences[0].front()->index, 2);
  ASSERT_EQ(ins.goal_sequences[0].back()->index, 4);
  ASSERT_EQ(ins.goal_sequences[1].front()->index, 5);
  ASSERT_EQ(ins.goal_sequences[1].back()->index, 7);

  Config goals({ins.G.U[4], ins.G.U[7]});
  goals.goal_indices = {3, 3};
  ASSERT_EQ(ins.goals, goals);
}
