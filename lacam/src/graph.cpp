#include "../include/graph.hpp"

Vertex::Vertex(int _id, int _index)
    : id(_id), index(_index), neighbor(Vertices())
{
}

std::vector<Orientation> Orientation::adjacent() const
{
  switch (value) {
    case UP:
      return {Orientation::LEFT, Orientation::UP, Orientation::RIGHT};
    case LEFT:
      return {Orientation::UP, Orientation::LEFT, Orientation::DOWN};
    case DOWN:
      return {Orientation::LEFT, Orientation::DOWN, Orientation::RIGHT};
    case RIGHT:
      return {Orientation::UP, Orientation::RIGHT, Orientation::DOWN};
    default:
      return {};
  }
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
      auto v = new Vertex(V.size(), index);
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

uint ConfigHasher::operator()(const Config& C) const
{
  uint location_hash = C.size();
  for (auto& v : C) {
    location_hash ^=
        v->id + 0x9e3779b9 + (location_hash << 6) + (location_hash >> 2);
  }
  uint indices_hash = C.goal_indices.size();
  for (auto& idx : C.goal_indices) {
    indices_hash ^=
        idx + 0x9e3779b9 + (indices_hash << 6) + (indices_hash >> 2);
  }
  return hash_combine(location_hash, indices_hash);
}

std::ostream& operator<<(std::ostream& os, const Vertex* v)
{
  os << v->index;
  return os;
}

std::ostream& operator<<(std::ostream& os, const Config& c)
{
  os << "{ ";
  std::copy(c.begin(), c.end(), std::ostream_iterator<Vertex*>(os, " "));
  os << "} ";
  os << "{ ";
  std::copy(c.goal_indices.begin(), c.goal_indices.end(),
            std::ostream_iterator<int>(os, " "));
  os << "}";
  return os;
}
