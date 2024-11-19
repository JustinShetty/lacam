#include <lacam.hpp>

#include "gtest/gtest.h"

static bool VERBOSITY = 0;

TEST(Instance, initialize)
{
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto ins = Instance(scen_filename, map_filename, 3);

  ASSERT_TRUE(ins.is_valid(VERBOSITY));

  ASSERT_EQ(ins.starts.size(), 3);
  ASSERT_EQ(ins.goal_sequences.size(), 3);
  ASSERT_EQ(ins.starts[0]->v->index, 203);
  ASSERT_EQ(ins.goal_sequences[0].back()->v->index, 583);
}

TEST(Instance, sequence)
{
  const auto map_filename = "./assets/empty-8-8.map";
  const auto ins = Instance(map_filename, {0, 1}, {{2, 3, 4}, {5, 6, 7}});

  ASSERT_EQ(ins.starts.size(), 2);
  ASSERT_EQ(ins.goal_sequences.size(), 2);
  ASSERT_EQ(ins.starts[0]->v->index, 0);
  ASSERT_EQ(ins.goal_sequences[0].back()->v->index, 4);

  ASSERT_EQ(ins.goal_sequences.size(), 2);
  ASSERT_EQ(ins.goal_sequences[0].front()->v->index, 2);
  ASSERT_EQ(ins.goal_sequences[0].back()->v->index, 4);
  ASSERT_EQ(ins.goal_sequences[1].front()->v->index, 5);
  ASSERT_EQ(ins.goal_sequences[1].back()->v->index, 7);
}

TEST(Instance, orientation)
{
  const std::string map_filename = "./tests/assets/2x2.map";
  const auto G = std::make_shared<Graph>(map_filename);
  const std::vector<StatePtr> starts = {
      G->NewState(G->U[0], 0, Orientation::X_MINUS),
      G->NewState(G->U[3], 0, Orientation::X_PLUS),
  };
  const std::vector<std::vector<StatePtr>> goals{
      {G->NewState(G->U[3], 0, Orientation::Y_PLUS)},
      {G->NewState(G->U[0], 0, Orientation::Y_MINUS)},
  };
  const Instance ins(G, starts, goals);
  ASSERT_TRUE(ins.is_valid(VERBOSITY));

  auto starts2 = starts;
  starts2[0]->o = Orientation::NONE;
  const Instance ins2(G, starts2, goals);
  ASSERT_FALSE(ins2.is_valid(VERBOSITY));

  auto goals2 = goals;
  goals2[1][0]->o = Orientation::NONE;
  const Instance ins3(G, starts, goals2);
  ASSERT_FALSE(ins3.is_valid(VERBOSITY));
}
