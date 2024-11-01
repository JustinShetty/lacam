#include "../include/instance.hpp"

#include <set>

Instance::Instance(const std::string& map_filename,
                   const std::vector<int>& start_indexes,
                   const std::vector<int>& goal_indexes,
                   const bool _consider_orientation)
    : G(map_filename),
      starts(Config()),
      goals(Config()),
      N(start_indexes.size()),
      consider_orientation(_consider_orientation)
{
  for (auto k : start_indexes) starts.push_back(G.U[k], 0);
  for (auto k : goal_indexes) {
    auto vp = G.U[k];
    goals.push_back(vp, 0);
    goal_sequences.push_back(std::vector<Vertex*>{vp});
  }
  starts.goal_indices = calculate_goal_indices(starts, starts);
}

Instance::Instance(const std::string& map_filename,
                   const std::vector<int>& start_indexes,
                   const std::vector<std::vector<int>>& goal_index_sequences,
                   const bool _consider_orientation)
    : G(map_filename),
      starts(Config()),
      goals(Config()),
      N(start_indexes.size()),
      consider_orientation(_consider_orientation)
{
  for (auto k : start_indexes) starts.push_back(G.U[k], 0);
  for (auto goal_sequence : goal_index_sequences) {
    std::vector<Vertex*> as_vertices;
    for (auto k : goal_sequence) as_vertices.push_back(G.U[k]);
    goal_sequences.push_back(as_vertices);
    goals.push_back(as_vertices.back(), as_vertices.size() - 1);
  }
  starts.goal_indices = calculate_goal_indices(starts, starts);
}

// for load instance
static const std::regex r_instance =
    std::regex(R"(\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+)");

Instance::Instance(const std::string& scen_filename,
                   const std::string& map_filename, const int _N,
                   const bool _consider_orientation)
    : G(Graph(map_filename)),
      starts(Config()),
      goals(Config()),
      N(_N),
      consider_orientation(_consider_orientation)
{
  // load start-goal pairs
  std::ifstream file(scen_filename);
  if (!file) {
    info(0, 0, scen_filename, " is not found");
    return;
  }
  std::string line;
  std::smatch results;

  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

    if (std::regex_match(line, results, r_instance)) {
      auto x_s = std::stoi(results[1].str());
      auto y_s = std::stoi(results[2].str());
      auto x_g = std::stoi(results[3].str());
      auto y_g = std::stoi(results[4].str());
      if (x_s < 0 || G.width <= x_s || x_g < 0 || G.width <= x_g) continue;
      if (y_s < 0 || G.height <= y_s || y_g < 0 || G.height <= y_g) continue;
      auto s = G.U[G.width * y_s + x_s];
      auto g = G.U[G.width * y_g + x_g];
      if (s == nullptr || g == nullptr) continue;
      starts.push_back(s, 0);
      goals.push_back(g, 0);
      goal_sequences.push_back(std::vector<Vertex*>{g});
    }

    if (starts.size() == N) break;
  }
  starts.goal_indices = calculate_goal_indices(starts, starts);
}

Instance::Instance(const std::string& map_filename, std::mt19937* MT,
                   const int _N, const bool _consider_orientation)
    : G(Graph(map_filename)),
      starts(Config()),
      goals(Config()),
      N(_N),
      consider_orientation(_consider_orientation)
{
  // random assignment
  const auto K = G.size();

  // set starts
  auto s_indexes = std::vector<int>(K);
  std::iota(s_indexes.begin(), s_indexes.end(), 0);
  std::shuffle(s_indexes.begin(), s_indexes.end(), *MT);
  int i = 0;
  while (true) {
    if (i >= K) return;
    starts.push_back(G.V[s_indexes[i]], 0);
    if (starts.size() == N) break;
    ++i;
  }

  // set goals
  auto g_indexes = std::vector<int>(K);
  std::iota(g_indexes.begin(), g_indexes.end(), 0);
  std::shuffle(g_indexes.begin(), g_indexes.end(), *MT);
  int j = 0;
  while (true) {
    if (j >= K) return;
    auto vp = G.V[g_indexes[j]];
    goals.push_back(vp, 0);
    goal_sequences.push_back(std::vector<Vertex*>{vp});
    if (goals.size() == N) break;
    ++j;
  }

  starts.goal_indices = calculate_goal_indices(starts, starts);
}

bool Instance::is_valid(const int verbose) const
{
  if (N != starts.size() || N != goals.size()) {
    info(1, verbose, "invalid N, check instance");
    return false;
  }

  return true;
}

int Instance::get_total_goals() const
{
  int total_goals = 0;
  for (const auto& goals : goal_sequences) {
    total_goals += goals.size();
  }
  return total_goals;
}

std::vector<int> Instance::calculate_goal_indices(const Config& c,
                                                  const Config& c_prev) const
{
  auto goal_indices = c_prev.goal_indices;
  for (size_t i = 0; i < N; ++i) {
    const auto current_location = c[i];
    const auto goal_seq = goal_sequences[i];
    auto& goal_idx = goal_indices[i];
    const auto next_goal = goal_seq[goal_indices[i]];
    if (current_location == next_goal && goal_idx < (int)goal_seq.size()) {
      goal_idx += 1;
    }
  }
  return goal_indices;
}