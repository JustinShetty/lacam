#include "../include/graph.hpp"

// initialize static variable
std::unordered_map<State, StatePtr, StateHasher> State::state_cache;

Vertex::Vertex(int _x, int _y, int _id, int _index)
    : x(_x), y(_y), id(_id), index(_index), neighbor(Vertices())
{
}

std::vector<Orientation> Orientation::adjacent() const
{
  switch (value) {
    case Y_PLUS:
      return {Orientation::X_MINUS, Orientation::X_PLUS};
    case Y_MINUS:
      return {Orientation::X_MINUS, Orientation::X_PLUS};
    case X_MINUS:
      return {Orientation::Y_MINUS, Orientation::Y_PLUS};
    case X_PLUS:
      return {Orientation::Y_MINUS, Orientation::Y_PLUS};
    default:
      return {};
  }
}

std::ostream& operator<<(std::ostream& os, const Orientation& o)
{
  switch (o) {
    case Orientation::Y_PLUS:
      os << "Y_PLUS";
      break;
    case Orientation::Y_MINUS:
      os << "Y_MINUS";
      break;
    case Orientation::X_MINUS:
      os << "X_MINUS";
      break;
    case Orientation::X_PLUS:
      os << "X_PLUS";
      break;
    default:
      os << "NONE";
      break;
  }
  return os;
}

void State::gen_neighbors()
{
  for (auto adjacent_orientation : o.adjacent()) {
    auto s = State::NewState(v, goal_index, adjacent_orientation);
    neighbors.push_back(s);
    in_neighbors.push_back(s);
  }
  for (auto u : v->neighbor) {
    auto candidate = State::NewState(u, goal_index, o);
    if (o == Orientation::Y_MINUS) {
      if (u->y == v->y - 1) neighbors.push_back(candidate);
      if (u->y == v->y + 1) in_neighbors.push_back(candidate);
    } else if (o == Orientation::Y_PLUS) {
      if (u->y == v->y + 1) neighbors.push_back(candidate);
      if (u->y == v->y - 1) in_neighbors.push_back(candidate);
    } else if (o == Orientation::X_MINUS) {
      if (u->x == v->x - 1) neighbors.push_back(candidate);
      if (u->x == v->x + 1) in_neighbors.push_back(candidate);
    } else if (o == Orientation::X_PLUS) {
      if (u->x == v->x + 1) neighbors.push_back(candidate);
      if (u->x == v->x - 1) in_neighbors.push_back(candidate);
    } else {
      neighbors.push_back(candidate);
    }
  }
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
  return hash_two_ints(s.v == nullptr ? 0 : s.v->id, hash_two_ints(s.o, s.goal_index));
}

uint StatePtrHasher::operator()(const StatePtr& s) const
{
  return StateHasher()(*s);
}

uint ConfigHasher::operator()(const Config& c) const
{
  StateHasher state_hasher;
  uint hash = c.size();
  for (auto& s : c) {
    hash = hash_two_ints(hash, state_hasher(*s));
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
