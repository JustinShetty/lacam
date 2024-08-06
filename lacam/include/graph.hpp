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
// using Config = std::vector<Vertex*>;  // locations for all agents

class Config : public std::vector<Vertex*> {
private:
  int additionalVariable;

public:
  Config() : additionalVariable(0) {}

  Config(int size, Vertex* value = nullptr, int additionalValue = 0)
      : std::vector<Vertex*>(size, value), additionalVariable(additionalValue) {}

  Config(std::initializer_list<Vertex*> il, int additionalValue = 0)

  int getAdditionalVariable() const {
    return additionalVariable;
  }

  void setAdditionalVariable(int value) {
    additionalVariable = value;
  }
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

bool enough_goals_reached(
  const Config& C1,
  const Config& C2,
  int threshold);

// hash function of configuration
// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct ConfigHasher {
  uint operator()(const Config& C) const;
};

std::ostream& operator<<(std::ostream& os, const Vertex* v);
