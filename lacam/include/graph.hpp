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
    X_MINUS = 1,
    X_PLUS = 2,
    Y_MINUS = 3,
    Y_PLUS = 4,
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

struct State;
using StatePtr = State*;
struct State
{
  Vertex* v;
  Orientation o;
  int goal_index;
  int pose_id;

  friend struct Graph;

  inline std::vector<StatePtr> get_neighbors()
  {
    if (neighbors.empty()) gen_neighbors();
    return neighbors;
  }

  inline std::vector<StatePtr> get_in_neighbors()
  {
    if (in_neighbors.empty()) gen_neighbors();
    return in_neighbors;
  }

  bool operator==(const State& other) const
  {
    return v == other.v && o == other.o && goal_index == other.goal_index;
  }
  bool operator!=(const State& other) const { return !(*this == other); }

private:
  State(Vertex* _v, int _goal_index, Orientation _o = Orientation::NONE)
      : v(_v),
        o(_o),
        goal_index(_goal_index),
        pose_id(v == nullptr ? 0 : Orientation::NUM_ORIENTATIONS * v->id + o),
        neighbors(),
        in_neighbors()
  {
  }
  State() : State(nullptr, 0) {}

  std::vector<StatePtr> neighbors;
  std::vector<StatePtr> in_neighbors;
  void gen_neighbors();
};

class Config
{
public:
  Config() : data() {}
  Config(const int N) : data(N) {}
  Config(const std::initializer_list<StatePtr> states) : data(states) {}
  Config(const std::vector<StatePtr>& states) : data(states) {}

  size_t size() const { return data.size(); }

  bool operator==(const Config& other) const { return data == other.data; }
  bool operator!=(const Config& other) const { return !(*this == other); }

  StatePtr& operator[](size_t i) { return data[i]; }
  StatePtr operator[](size_t i) const { return data[i]; }

  auto begin() { return data.begin(); }
  auto begin() const { return data.begin(); }
  auto end() { return data.end(); }
  auto end() const { return data.end(); }

  void push_back(StatePtr s) { data.push_back(s); }

  bool enough_goals_reached(int threshold) const
  {
    int count = 0;
    for (size_t i = 0; i < size(); ++i) {
      count += data[i]->goal_index;
      if (count >= threshold) return true;
    }
    return false;
  }

private:
  std::vector<StatePtr> data;
};

// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct StateHasher {
  uint operator()(const State& s) const;
};
struct StatePtrHasher {
  uint operator()(const StatePtr& s) const;
};
struct ConfigHasher {
  uint operator()(const Config& c) const;
};

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

  static StatePtr NewState(Vertex* _v, int _goal_index,
                           Orientation _o = Orientation::NONE)
  {
    State s(_v, _goal_index, _o);
    if (states.find(s) == states.end()) {
      states[s] = new State(_v, _goal_index, _o);
    }
    return states[s];
  }

  static StatePtr NewState() { return NewState(nullptr, 0); }

private:
  static std::unordered_map<State, StatePtr, StateHasher> states;
};

std::ostream& operator<<(std::ostream& os, const Vertex* v);
std::ostream& operator<<(std::ostream& os, const Orientation& o);
std::ostream& operator<<(std::ostream& os, const State& s);
std::ostream& operator<<(std::ostream& os, const StatePtr& s);
std::ostream& operator<<(std::ostream& os, const Config& c);
