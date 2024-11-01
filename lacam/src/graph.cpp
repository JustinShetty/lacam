#include "../include/graph.hpp"

Vertex::Vertex(int _x, int _y, int _id, int _index)
    : x(_x), y(_y), id(_id), index(_index), neighbor(Vertices())
{
}

std::vector<Orientation> Orientation::adjacent() const
{
  switch (value) {
    case UP:
      return {Orientation::LEFT, Orientation::RIGHT};
    case LEFT:
      return {Orientation::UP, Orientation::DOWN};
    case DOWN:
      return {Orientation::LEFT, Orientation::RIGHT};
    case RIGHT:
      return {Orientation::UP, Orientation::DOWN};
    default:
      return {};
  }
}

std::ostream& operator<<(std::ostream& os, const Orientation& o)
{
  switch (o) {
    case Orientation::UP:
      os << "UP";
      break;
    case Orientation::LEFT:
      os << "LEFT";
      break;
    case Orientation::DOWN:
      os << "DOWN";
      break;
    case Orientation::RIGHT:
      os << "RIGHT";
      break;
    default:
      os << "NONE";
      break;
  }
  return os;
}

State::State(Vertex* _v, int _goal_index, Orientation _o)
    : v(_v),
      o(_o),
      goal_index(_goal_index),
      neighbors(),
      neighbors_generated(false)
{
}

void State::gen_neighbors()
{
  neighbors = std::vector<State>();
  for (auto oa : o.adjacent()) {
    neighbors.emplace_back(v, goal_index, oa);
  }
  for (auto u : v->neighbor) {
    if (o == Orientation::UP && u->y != v->y + 1)
      continue;
    else if (o == Orientation::DOWN && u->y != v->y - 1)
      continue;
    else if (o == Orientation::LEFT && u->x != v->x - 1)
      continue;
    else if (o == Orientation::RIGHT && u->x != v->x + 1)
      continue;
    neighbors.push_back({u, goal_index, o});
  }
  neighbors_generated = true;
}

std::ostream& operator<<(std::ostream& os, const State& s)
{
  os << "State((" << s.v->x << ", " << s.v->y << ") " << s.o << " "
     << s.goal_index << ")";
  return os;
}

Graph::Graph() : V(Vertices()), width(0), height(0) {}
Graph::~Graph()
{
  for (auto& v : V)
    if (v != nullptr) delete v;
  V.clear();
}

// to load graph
static const std::regex r_height = std::regex(R"(height\s(\d+))");
static const std::regex r_width = std::regex(R"(width\s(\d+))");
static const std::regex r_map = std::regex(R"(map)");

Graph::Graph(const std::string& filename) : V(Vertices()), width(0), height(0)
{
  std::ifstream file(filename);
  if (!file) {
    std::cout << "file " << filename << " is not found." << std::endl;
    return;
  }
  std::string line;
  std::smatch results;

  // read fundamental graph parameters
  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

    if (std::regex_match(line, results, r_height)) {
      height = std::stoi(results[1].str());
    }
    if (std::regex_match(line, results, r_width)) {
      width = std::stoi(results[1].str());
    }
    if (std::regex_match(line, results, r_map)) break;
  }

  U = Vertices(width * height, nullptr);

  // create vertices
  int y = 0;
  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();
    for (int x = 0; x < width; ++x) {
      char s = line[x];
      if (s == 'T' or s == '@') continue;  // object
      auto index = width * y + x;
      auto v = new Vertex(x, y, V.size(), index);
      V.push_back(v);
      U[index] = v;
    }
    ++y;
  }
  file.close();

  // create edges
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      auto v = U[width * y + x];
      if (v == nullptr) continue;
      // left
      if (x > 0) {
        auto u = U[width * y + (x - 1)];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      // right
      if (x < width - 1) {
        auto u = U[width * y + (x + 1)];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      // up
      if (y < height - 1) {
        auto u = U[width * (y + 1) + x];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      // down
      if (y > 0) {
        auto u = U[width * (y - 1) + x];
        if (u != nullptr) v->neighbor.push_back(u);
      }
    }
  }
}

int Graph::size() const { return V.size(); }

uint StateHasher::operator()(const State& s) const
{
  return hash_two_ints(s.v->id, hash_two_ints(s.o, s.goal_index));
}

uint ConfigHasher::operator()(const Config& c) const
{
  StateHasher state_hasher;
  uint hash = c.size();
  for (auto& s : c) {
    hash = hash_two_ints(hash, state_hasher(s));
  }
  return hash;
}

std::ostream& operator<<(std::ostream& os, const Vertex* v)
{
  os << v->index;
  return os;
}

std::ostream& operator<<(std::ostream& os, const Config& c)
{
  os << "{ ";
  for (const auto& s : c) {
    os << s << " ";
  }
  os << "}";
  return os;
}
