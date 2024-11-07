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
  Orientation y_plus = Orientation::Y_PLUS;
  Orientation y_minus = Orientation::Y_MINUS;
  Orientation x_minus = Orientation::X_MINUS;
  Orientation x_plus = Orientation::X_PLUS;
  ASSERT_EQ(none.adjacent(), std::vector<Orientation>());
  ASSERT_EQ(y_plus.adjacent(), std::vector<Orientation>({x_minus, x_plus}));
  ASSERT_EQ(y_minus.adjacent(), std::vector<Orientation>({x_minus, x_plus}));
  ASSERT_EQ(x_minus.adjacent(), std::vector<Orientation>({y_minus, y_plus}));
  ASSERT_EQ(x_plus.adjacent(), std::vector<Orientation>({y_minus, y_plus}));
}

TEST(State, get_neighbors)
{
  const std::string filename = "./assets/random-32-32-10.map";
  auto G = Graph(filename);
  auto s = State(G.V[0], 0, Orientation::Y_PLUS);
  auto neighbors = s.get_neighbors();
  std::vector<State> expected_neighbors = {
      State(G.V[0], 0, Orientation::X_MINUS),
      State(G.V[0], 0, Orientation::X_PLUS),
      State(G.U[32], 0, Orientation::Y_PLUS)};
  ASSERT_EQ(neighbors.size(), expected_neighbors.size());
  for (size_t i = 0; i < neighbors.size(); ++i) {
    ASSERT_EQ(neighbors[i], expected_neighbors[i]);
  }
}

TEST(State, get_in_neighbors)
{
  const std::string filename = "./assets/random-32-32-10.map";
  auto G = Graph(filename);
  auto s = State(G.V[1], 0, Orientation::Y_MINUS);
  auto in_neighbors = s.get_in_neighbors();
  std::vector<State> expected_neighbors = {
      State(G.V[1], 0, Orientation::X_MINUS),
      State(G.V[1], 0, Orientation::X_PLUS),
      State(G.U[33], 0, Orientation::Y_MINUS)};
  ASSERT_EQ(in_neighbors.size(), expected_neighbors.size());
  for (size_t i = 0; i < in_neighbors.size(); ++i) {
    ASSERT_EQ(in_neighbors[i], expected_neighbors[i]);
  }
}
