#include "../include/dist_table.hpp"

#include <algorithm>

DistTableMultiGoal::DistTableMultiGoal(const Instance* _ins)
    : ins(_ins), table(), OPEN()
{
  setup(ins);
}

void DistTableMultiGoal::setup(const Instance* ins)
{
  // initialize search queues and table values for goals
  for (size_t i = 0; i < ins->N; i++) {
    table.push_back(std::unordered_map<State, int, StateHasher>());
    OPEN.push_back(std::queue<State>());
    const auto& goal_seq = ins->goal_sequences[i];
    for (size_t j = 0; j < goal_seq.size(); j++) {
      auto g = goal_seq[j];
      OPEN[i].push(g);
      table[i][g] = 0;
    }
  }
}

int DistTableMultiGoal::get(int agent_id, const State& from)
{
  // goal_index can be past the end to signify we've already reached the last
  // goal, but when we want to use the index we need to cap it at the last goal
  auto goal_index = std::min(from.goal_index,
                             (int)(ins->goal_sequences[agent_id].size() - 1));
  auto key = State(from.v, from.o, goal_index);

  if (table[agent_id].find(key) != table[agent_id].end()) {
    return table[agent_id][key];
  }

  /*
   * BFS with lazy evaluation
   * c.f., Reverse Resumable A*
   * https://www.aaai.org/Papers/AIIDE/2005/AIIDE05-020.pdf
   */

  const auto K = ins->G->size();
  const auto next_goal = ins->goal_sequences[agent_id][goal_index];
  while (!OPEN[agent_id].empty()) {
    auto n = OPEN[agent_id].front();
    OPEN[agent_id].pop();
    if (table[agent_id].find(n) == table[agent_id].end()) {
      table[agent_id][n] = K;
    }
    auto d_n = table[agent_id][n];
    const auto neighbors = n.get_neighbors();
    for (const auto& m : neighbors) {
      if (table[agent_id].find(m) == table[agent_id].end()) {
        table[agent_id][m] = K;
      }
      auto d_m = table[agent_id][m];
      if (d_n + 1 >= d_m) continue;
      table[agent_id][m] = d_n + 1;
      OPEN[agent_id].push(m);
    }
    if (n == from) return d_n;
  }
  return K;
}
