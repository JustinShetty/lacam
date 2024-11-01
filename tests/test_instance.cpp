#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(Instance, initialize)
{
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto ins = Instance(scen_filename, map_filename, 3);

  ASSERT_TRUE(ins.is_valid(0));

  ASSERT_EQ(ins.starts.size(), 3);
  ASSERT_EQ(ins.goal_sequences.size(), 3);
  ASSERT_EQ(ins.starts[0].v->index, 203);
  ASSERT_EQ(ins.goal_sequences[0].back().v->index, 583);
}

TEST(Instance, sequence)
{
  const auto map_filename = "./assets/empty-8-8.map";
  const auto ins = Instance(map_filename, {0, 1}, {{2, 3, 4}, {5, 6, 7}});

  ASSERT_EQ(ins.starts.size(), 2);
  ASSERT_EQ(ins.goal_sequences.size(), 2);
  ASSERT_EQ(ins.starts[0].v->index, 0);
  ASSERT_EQ(ins.goal_sequences[0].back().v->index, 4);

  ASSERT_EQ(ins.goal_sequences.size(), 2);
  ASSERT_EQ(ins.goal_sequences[0].front().v->index, 2);
  ASSERT_EQ(ins.goal_sequences[0].back().v->index, 4);
  ASSERT_EQ(ins.goal_sequences[1].front().v->index, 5);
  ASSERT_EQ(ins.goal_sequences[1].back().v->index, 7);
}
