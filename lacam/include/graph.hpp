/*
 * graph definition
 */
#pragma once

#include "utils.hpp"

struct Vertex {
  const int x;
  const int y;
  const int id;     // index for V in Graph
  const int index;  // index for U (width * y + x) in Graph
  std::vector<Vertex*> neighbor;

  Vertex(int _x, int y_, int _id, int _index);
};
using Vertices = std::vector<Vertex*>;

class Orientation
{
public:
  enum Value : uint8_t {
    NONE = 0,
    UP = 1,
    LEFT = 2,
    DOWN = 3,
    RIGHT = 4,
    NUM_ORIENTATIONS = 5
  };

  constexpr Orientation(Value v) : value(v) {}
  constexpr bool operator==(const Orientation::Value& v) const
  {
    return value == v;
  }
  constexpr bool operator==(const Orientation& o) const
  {
    return value == o.value;
  }
  constexpr bool operator!=(const Orientation::Value& v) const
  {
    return value != v;
  }
  constexpr bool operator!=(const Orientation& o) const
  {
    return value != o.value;
  }

  constexpr operator Value() const { return value; }
  explicit operator bool() const = delete;

  std::vector<Orientation> adjacent() const;

private:
  Value value;
};

std::ostream& operator<<(std::ostream& os, const Orientation& o);

class State
{
public:
  Vertex* v;
  Orientation o;
  int goal_index;

  State() : v(nullptr), o(Orientation::NONE), goal_index(0) {}
  State(Vertex* _v, Orientation _o, int _goal_index);

  std::vector<State> get_neighbors();

  bool operator==(const State& other) const
  {
    return v == other.v && o == other.o && goal_index == other.goal_index;
  }
  bool operator!=(const State& other) const { return !(*this == other); }

private:
  std::vector<State> neighbors;
  bool neighbors_generated;
  void gen_neighbors();
};

std::ostream& operator<<(std::ostream& os, const State& s);

class Config
{
public:
  Config() : data() {}
  Config(const int N) : data(N) {}
  Config(const int N, State s) : data(N, s) {}
  Config(const std::initializer_list<State> states) : data(states) {}
  Config(const std::vector<State>& states) : data(states) {}

  size_t size() const { return data.size(); }

  bool operator==(const Config& other) const { return data == other.data; }
  bool operator!=(const Config& other) const { return !(*this == other); }

  State& operator[](size_t i) { return data[i]; }
  State operator[](size_t i) const { return data[i]; }

  auto begin() { return data.begin(); }
  auto begin() const { return data.begin(); }
  auto end() { return data.end(); }
  auto end() const { return data.end(); }

  void push_back(State s) { data.push_back(s); }

  bool enough_goals_reached(int threshold) const
  {
    int count = 0;
    for (size_t i = 0; i < size(); ++i) {
      count += data[i].goal_index;
      if (count >= threshold) return true;
    }
    return false;
  }

private:
  std::vector<State> data;
};

std::ostream& operator<<(std::ostream& os, const Config& c);

struct Graph {
  Vertices V;  // without nullptr
  Vertices U;  // with nullptr, i.e., |U| = width * height
  int width;   // grid width
  int height;  // grid height
  Graph();
  Graph(const Graph& g);
  Graph(const std::string& filename);  // taking map filename
  ~Graph();

  int size() const;  // the number of vertices, |V|

private:
  bool garbage_collect = true;
};

// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct StateHasher {
  uint operator()(const State& s) const;
};
struct ConfigHasher {
  uint operator()(const Config& c) const;
};

std::ostream& operator<<(std::ostream& os, const Vertex* v);
