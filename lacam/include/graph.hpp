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

class Orientation
{
public:
  enum Value : int8_t {
    NONE = -1,
    UP = 0,
    LEFT = 1,
    DOWN = 2,
    RIGHT = 3,
    // NUM_ORIENTATIONS = 4
  };

  constexpr Orientation(Value v) : value(v) {}
  constexpr bool operator==(const Orientation& o) const
  {
    return value == o.value;
  }
  constexpr bool operator!=(const Orientation& o) const
  {
    return value != o.value;
  }

  std::vector<Orientation> adjacent() const;

private:
  Value value;
};

class Config : public std::vector<Vertex*>
{
public:
  Config() : goal_indices(), orientations(), data() {}
  Config(const int N, Vertex* v)
      : goal_indices(N, 0), orientations(), data(N, v)
  {
  }
  Config(const std::initializer_list<Vertex*> vertices)
      : goal_indices(vertices.size(), 0), orientations(), data(vertices)
  {
  }
  Config(const std::initializer_list<Vertex*> vertices,
         const std::initializer_list<int> goal_indices)
      : goal_indices(goal_indices), orientations(), data(vertices)
  {
  }

  size_t size() const { return data.size(); }

  bool operator==(const Config& other) const
  {
    return data == other.data && goal_indices == other.goal_indices;
  }
  bool operator!=(const Config& other) const { return !(*this == other); }

  Vertex*& operator[](size_t i) { return data[i]; }
  Vertex* operator[](size_t i) const { return data[i]; }

  auto begin() { return data.begin(); }
  auto begin() const { return data.begin(); }
  auto end() { return data.end(); }
  auto end() const { return data.end(); }

  void push_back(Vertex* v, int goal_index)
  {
    data.push_back(v);
    goal_indices.push_back(goal_index);
  }

  bool enough_goals_reached(int threshold) const
  {
    int count = 0;
    for (size_t i = 0; i < size(); ++i) {
      count += goal_indices[i];
      if (count >= threshold) return true;
    }
    return false;
  }

  std::vector<int> goal_indices;
  std::vector<Orientation> orientations;

private:
  std::vector<Vertex*> data;
};

std::ostream& operator<<(std::ostream& os, const Config& c);

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

// hash function of configuration
// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct ConfigHasher {
  uint operator()(const Config& C) const;
};

std::ostream& operator<<(std::ostream& os, const Vertex* v);
