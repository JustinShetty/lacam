/*
 * graph definition
 */
#pragma once
#include "utils.hpp"

struct Vertex {
  const int id;     // index for V in Graph
  const int index;  // index for U (width * y + x) in Graph
  std::vector<Vertex*> neighbor;

  Vertex(int _id, int _index);
};
using Vertices = std::vector<Vertex*>;
class Config : public std::vector<Vertex*>
{
public:
  Config() : std::vector<Vertex*>(), goal_indices() {}
  Config(const int N, Vertex* v)
      : std::vector<Vertex*>(N, v), goal_indices(N, 0)
  {
  }
  Config(const std::initializer_list<Vertex*> init_list)
      : std::vector<Vertex*>(init_list), goal_indices(init_list.size(), 0)
  {
  }

  bool operator==(const Config& C) const
  {
    if (this->size() != C.size()) return false;
    for (int i = 0; i < this->size(); ++i) {
      if (this->at(i) != C.at(i)) return false;
    }
    for (int i = 0; i < goal_indices.size(); i++) {
      if (goal_indices[i] != C.goal_indices[i]) return false;
    }
    return true;
  }

  void push_back(Vertex* v)
  {
    std::vector<Vertex*>::push_back(v);
    goal_indices.push_back(0);
  }

  std::vector<int> goal_indices;
};

struct Graph {
  Vertices V;  // without nullptr
  Vertices U;  // with nullptr, i.e., |U| = width * height
  int width;   // grid width
  int height;  // grid height
  Graph();
  Graph(const std::string& filename);  // taking map filename
  ~Graph();

  int size() const;  // the number of vertices, |V|
};

bool is_same_config(
    const Config& C1,
    const Config& C2);  // check equivalence of two configurations

bool enough_goals_reached(const Config& C1, const Config& C2, int threshold);

// hash function of configuration
// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct ConfigHasher {
  uint operator()(const Config& C) const;
};

std::ostream& operator<<(std::ostream& os, const Vertex* v);
