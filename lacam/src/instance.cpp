#include "../include/instance.hpp"

#include <set>

Instance::Instance(const std::string& map_filename,
                   const std::vector<int>& start_indexes,
                   const std::vector<int>& goal_indexes)
    : G(std::shared_ptr<Graph>(new Graph(map_filename))),
      starts(Config()),
      N(start_indexes.size())
{
  for (auto k : start_indexes) {
    starts.push_back(State(G->U[k], 0));
  }
  for (auto k : goal_indexes) {
    goal_sequences.push_back(std::vector<State>{State(G->U[k], 0)});
  }
  update_goal_indices(starts, starts);
}

Instance::Instance(const std::string& map_filename,
                   const std::vector<int>& start_indexes,
                   const std::vector<std::vector<int>>& goal_index_sequences)
    : G(std::shared_ptr<Graph>(new Graph(map_filename))),
      starts(Config()),
      N(start_indexes.size())
{
  for (auto k : start_indexes) {
    starts.push_back(State(G->U[k], 0));
  }
  for (auto goal_sequence : goal_index_sequences) {
    std::vector<State> as_states;
    for (int k = 0; k < goal_sequence.size(); ++k) {
      as_states.push_back(State(G->U[goal_sequence[k]], k));
    }
    goal_sequences.push_back(as_states);
  }
  update_goal_indices(starts, starts);
}

Instance::Instance(const std::shared_ptr<Graph> _G,
                   const std::vector<State>& _starts,
                   const std::vector<std::vector<State>>& _goal_sequences)
    : G(_G), starts(_starts), goal_sequences(_goal_sequences), N(starts.size())
{
  update_goal_indices(starts, starts);
}

// for load instance
static const std::regex r_instance =
    std::regex(R"(\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+)");

Instance::Instance(const std::string& scen_filename,
                   const std::string& map_filename, const int _N)
    : G(std::shared_ptr<Graph>(new Graph(map_filename))),
      starts(Config()),
      N(_N)
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
      if (x_s < 0 || G->width <= x_s || x_g < 0 || G->width <= x_g) continue;
      if (y_s < 0 || G->height <= y_s || y_g < 0 || G->height <= y_g) continue;
      auto s = G->U[G->width * y_s + x_s];
      auto g = G->U[G->width * y_g + x_g];
      if (s == nullptr || g == nullptr) continue;
      starts.push_back(State(s, 0));
      goal_sequences.push_back(std::vector<State>{State(g, 0)});
    }

    if (starts.size() == N) break;
  }
  update_goal_indices(starts, starts);
}

Instance::Instance(const std::string& map_filename, std::mt19937* MT,
                   const int _N)
    : G(std::shared_ptr<Graph>(new Graph(map_filename))),
      starts(Config()),
      N(_N)
{
  // random assignment
  const auto K = G->size();

  // set starts
  auto s_indexes = std::vector<int>(K);
  std::iota(s_indexes.begin(), s_indexes.end(), 0);
  std::shuffle(s_indexes.begin(), s_indexes.end(), *MT);
  int i = 0;
  while (true) {
    if (i >= K) return;
    starts.push_back(State(G->V[s_indexes[i]], 0));
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
    goal_sequences.push_back(std::vector<State>{State(G->V[g_indexes[j]], 0)});
    if (goal_sequences.size() == N) break;
    ++j;
  }
  update_goal_indices(starts, starts);
}

bool Instance::is_valid(const int verbose) const
{
  if (N != starts.size() || N != goal_sequences.size()) {
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

void Instance::update_goal_indices(Config& c, const Config& c_prev) const
{
  for (size_t i = 0; i < N; ++i) {
    const auto current = c[i];
    const auto goal_seq = goal_sequences[i];
    auto goal_idx = c_prev[i].goal_index;
    if (goal_idx < goal_seq.size() && current == goal_seq[goal_idx]) {
      goal_idx += 1;
    }
    c[i] = State(current.v, goal_idx, current.o);
  }
}
