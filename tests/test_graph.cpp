#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(Graph, load_graph)
{
  const std::string filename = "./assets/random-32-32-10.map";
  auto G = Graph(filename);
  ASSERT_EQ(G.size(), 922);
  ASSERT_EQ(G.V[0]->neighbor.size(), 2);
  ASSERT_EQ(G.V[0]->neighbor[0]->id, 1);
  ASSERT_EQ(G.V[0]->neighbor[1]->id, 28);
  ASSERT_EQ(G.width, 32);
  ASSERT_EQ(G.height, 32);
}

TEST(Orientation, adjacent)
{
  Orientation none = Orientation::NONE;
  Orientation up = Orientation::UP;
  Orientation down = Orientation::DOWN;
  Orientation left = Orientation::LEFT;
  Orientation right = Orientation::RIGHT;
  ASSERT_EQ(none.adjacent(), std::vector<Orientation>({none}));
  ASSERT_EQ(up.adjacent(), std::vector<Orientation>({left, right}));
  ASSERT_EQ(down.adjacent(), std::vector<Orientation>({left, right}));
  ASSERT_EQ(left.adjacent(), std::vector<Orientation>({up, down}));
  ASSERT_EQ(right.adjacent(), std::vector<Orientation>({up, down}));
}

TEST(State, get_neighbors)
{
  const std::string filename = "./assets/random-32-32-10.map";
  auto G = Graph(filename);
  auto s = State(G.V[0], Orientation::UP, 0);
  auto neighbors = s.get_neighbors();
  std::vector<State> expected_neighbors = {State(G.V[0], Orientation::LEFT, 0),
                                           State(G.V[0], Orientation::RIGHT, 0),
                                           State(G.U[32], Orientation::UP, 0)};
  ASSERT_EQ(neighbors.size(), expected_neighbors.size());
  for (size_t i = 0; i < neighbors.size(); ++i) {
    ASSERT_EQ(neighbors[i], expected_neighbors[i]);
  }
}