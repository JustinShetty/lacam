#include "../include/dist_table.hpp"

DistTable::DistTable(const Instance* ins)
    : K(ins->G.V.size()), table(ins->N, std::vector<int>(K, K))
{
  setup(ins);
}

void DistTable::setup(const Instance* ins)
{
  for (size_t i = 0; i < ins->N; ++i) {
    OPEN.push_back(std::queue<Vertex*>());
    auto n = ins->goals[i];
    OPEN[i].push(n);
    table[i][n->id] = 0;
  }
}

int DistTable::get(int i, int v_id)
{
  if (table[i][v_id] < K) return table[i][v_id];

  /*
   * BFS with lazy evaluation
   * c.f., Reverse Resumable A*
   * https://www.aaai.org/Papers/AIIDE/2005/AIIDE05-020.pdf
   */

  while (!OPEN[i].empty()) {
    auto n = OPEN[i].front();
    OPEN[i].pop();
    const int d_n = table[i][n->id];
    for (auto& m : n->neighbor) {
      const int d_m = table[i][m->id];
      if (d_n + 1 >= d_m) continue;
      table[i][m->id] = d_n + 1;
      OPEN[i].push(m);
    }
    if (n->id == v_id) return d_n;
  }
  return K;
}

//
//
//
//
//
//
//
//
//
//

DistTableMultiGoal::DistTableMultiGoal(const Instance* ins)
    : K(ins->G.V.size()), table(), OPEN()
{
  setup(ins);
}

void DistTableMultiGoal::setup(const Instance* ins)
{
  // initialize all values to K
  for (size_t i = 0; i < ins->N; i++) {
    table.push_back(std::vector<std::vector<int>>(ins->goal_sequences[i].size(),
                                                  std::vector<int>(K, K)));
  }

  // initialize search queues and table values for goals
  for (size_t i = 0; i < ins->N; i++) {
    OPEN.push_back(
        std::vector<std::queue<Vertex*>>(ins->goal_sequences[i].size()));
    for (size_t j = 0; j < ins->goal_sequences[i].size(); j++) {
      auto g = ins->goal_sequences[i][j];
      OPEN[i][j].push(g);
      table[i][j][g->id] = 0;
    }
  }
}

int DistTableMultiGoal::get(int agent_id, int goal_index, int from_id)
{
  if (table[agent_id][goal_index][from_id] < K)
    return table[agent_id][goal_index][from_id];

  /*
    * BFS with lazy evaluation
    * c.f., Reverse Resumable A*
    * https://www.aaai.org/Papers/AIIDE/2005/AIIDE05-020.pdf
    */

  while (!OPEN[agent_id][goal_index].empty()) {
    auto n = OPEN[agent_id][goal_index].front();
    OPEN[agent_id][goal_index].pop();
    const int d_n = table[agent_id][goal_index][n->id];
    for (auto& m : n->neighbor) {
      const int d_m = table[agent_id][goal_index][m->id];
      if (d_n + 1 >= d_m) continue;
      table[agent_id][goal_index][m->id] = d_n + 1;
      OPEN[agent_id][goal_index].push(m);
    }
    if (n->id == from_id) return d_n;
  }
  return K;
}
