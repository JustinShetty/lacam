#include "../include/planner.hpp"

#include <algorithm>
#include <tuple>

using ExplorationKey = std::tuple<Config, std::vector<int>>;

Constraint::Constraint() : who(std::vector<int>()), where(Vertices()), depth(0)
{
}

Constraint::Constraint(Constraint* parent, int i, Vertex* v)
    : who(parent->who), where(parent->where), depth(parent->depth + 1)
{
  who.push_back(i);
  where.push_back(v);
}

Constraint::~Constraint() {};

Node::Node(Config _C, DistTable& D, std::vector<int> _goal_indices,
           Node* _parent)
    : C(_C),
      goal_indices(_goal_indices),
      parent(_parent),
      priorities(C.size(), 0),
      order(C.size(), 0),
      search_tree(std::queue<Constraint*>())
{
  search_tree.push(new Constraint());
  const auto N = C.size();

  // set priorities
  if (parent == nullptr) {
    // initialize
    for (size_t i = 0; i < N; ++i) priorities[i] = (float)D.get(i, C[i]) / N;
  } else {
    // dynamic priorities, akin to PIBT
    for (size_t i = 0; i < N; ++i) {
      if (D.get(i, C[i]) != 0) {
        priorities[i] = parent->priorities[i] + 1;
      } else {
        priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
      }
    }
  }

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int i, int j) { return priorities[i] > priorities[j]; });
}

Node::~Node()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

Planner::Planner(const Instance* _ins, const Deadline* _deadline,
                 std::mt19937* _MT, int _verbose, int _threshold,
                 bool _allow_following)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      threshold(_threshold),
      allow_following(_allow_following),
      N(ins->N),
      V_size(ins->G.size()),
      D(DistTable(ins)),
      C_next(Candidates(N, std::array<Vertex*, 5>())),
      tie_breakers(std::vector<float>(V_size, 0)),
      A(Agents(N, nullptr)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr))
{
}

struct ExplorationKeyHasher {
  uint operator()(const ExplorationKey& cai) const
  {
    Config C;
    std::vector<int> indices;
    std::tie(C, indices) = cai;
    uint config_hash = C.size();
    for (auto& v : C) {
      config_hash ^=
          v->id + 0x9e3779b9 + (config_hash << 6) + (config_hash >> 2);
    }
    uint indices_hash = indices.size();
    for (auto& idx : indices) {
      indices_hash ^=
          idx + 0x9e3779b9 + (indices_hash << 6) + (indices_hash >> 2);
    }
    return hash_combine(config_hash, indices_hash);
  }
};

Solution Planner::solve()
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart search");

  const bool multiple_goals = ins->goal_sequences[0].size() > 1;

  // setup agents
  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

  // setup search queues
  std::stack<Node*> OPEN;
  std::unordered_map<ExplorationKey, Node*, ExplorationKeyHasher> CLOSED;
  std::vector<Constraint*> GC;  // garbage collection of constraints

  // insert initial node
  auto initial_goal_indices = std::vector<int>(N, 0);
  auto S = new Node(ins->starts, D, initial_goal_indices);
  OPEN.push(S);
  CLOSED[ExplorationKey(S->C, S->goal_indices)] = S;

  // depth first search
  int loop_cnt = 0;
  std::vector<Config> solution;

  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;

    // do not pop here!
    S = OPEN.top();

    // check goal condition
    auto latest_goal_indices = S->goal_indices;
    for (auto i = 0; i < N; ++i) {
      const auto current_location = S->C[i];
      const auto goal_seq = ins->goal_sequences[i];
      auto& goal_idx = latest_goal_indices[i];
      const auto next_goal = goal_seq[latest_goal_indices[i]];
      if (current_location == next_goal && goal_idx < (int)goal_seq.size()) {
        goal_idx += 1;
      }
    }

    bool goals_exhausted = true;
    for (auto i = 0; i < N; i++) {
      if (latest_goal_indices[i] < (int)ins->goal_sequences[i].size()) {
        goals_exhausted = false;
        break;
      }
    }

    const auto threshold_met =
        enough_goals_reached(S->C, ins->goals, threshold);
    if (goals_exhausted && (multiple_goals || threshold_met)) {
      // backtrack
      while (S != nullptr) {
        solution.push_back(S->C);
        S = S->parent;
      }
      std::reverse(solution.begin(), solution.end());
      break;
    }

    // low-level search end
    if (S->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    // create successors at the low-level search
    auto M = S->search_tree.front();
    GC.push_back(M);
    S->search_tree.pop();
    if (M->depth < N) {
      auto i = S->order[M->depth];
      auto C = S->C[i]->neighbor;
      C.push_back(S->C[i]);
      if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);  // randomize
      for (auto u : C) S->search_tree.push(new Constraint(M, i, u));
    }

    // create successors at the high-level search
    if (!get_new_config(S, M)) continue;

    // create new configuration
    auto C = Config(N, nullptr);
    for (auto a : A) C[a->id] = a->v_next;

    // check explored list
    auto iter = CLOSED.find(ExplorationKey(C, latest_goal_indices));
    if (iter != CLOSED.end()) {
      OPEN.push(iter->second);
      continue;
    }

    // insert new search node
    auto S_new = new Node(C, D, latest_goal_indices, S);
    OPEN.push(S_new);
    CLOSED[ExplorationKey(S_new->C, S_new->goal_indices)] = S_new;
  }

  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\t",
       solution.empty() ? (OPEN.empty() ? "no solution" : "failed")
                        : "solution found",
       "\tloop_itr:", loop_cnt, "\texplored:", CLOSED.size());
  // memory management
  for (auto a : A) delete a;
  for (auto M : GC) delete M;
  for (auto p : CLOSED) delete p.second;

  return solution;
}

bool Planner::get_new_config(Node* S, Constraint* M)
{
  // setup cache
  for (auto a : A) {
    // clear previous cache
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }

    // set occupied now
    a->v_now = S->C[a->id];
    occupied_now[a->v_now->id] = a;
  }

  // add constraints
  for (auto k = 0; k < M->depth; ++k) {
    const auto i = M->who[k];        // agent
    const auto l = M->where[k]->id;  // loc

    // check vertex collision
    if (occupied_next[l] != nullptr) return false;

    if (allow_following) {
      // check swap collision
      auto l_pre = S->C[i]->id;
      if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
          occupied_next[l_pre]->id == occupied_now[l]->id) {
        return false;
      }
    } else {
      // check following conflict
      if (occupied_now[l] != nullptr && occupied_now[l] != A[i]) return false;
    }

    // set occupied_next
    A[i]->v_next = M->where[k];
    occupied_next[l] = A[i];
  }

  // perform PIBT
  for (auto k : S->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a)) return false;  // planning failure
  }
  return true;
}

bool Planner::funcPIBT(Agent* ai)
{
  if (allow_following) return funcPIBT_following(ai);
  return funcPIBT_no_following(ai, nullptr);
}

bool Planner::funcPIBT_following(Agent* ai)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // get candidates for next locations
  for (size_t k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;

  // sort, note: K + 1 is sufficient
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {
              return D.get(i, v) + tie_breakers[v->id] <
                     D.get(i, u) + tie_breakers[u->id];
            });

  for (size_t k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];

    // avoid swap conflicts with constraints
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // reserve next location
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // empty or stay
    if (ak == nullptr || u == ai->v_now) return true;

    // priority inheritance
    if (ak->v_next == nullptr && !funcPIBT_following(ak)) continue;

    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

bool Planner::funcPIBT_no_following(Agent* ai, Agent* aj)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // get candidates for next locations
  for (size_t k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  size_t num_candidates = K;
  if (aj == nullptr) {
    C_next[i][K] = ai->v_now;
    num_candidates++;
  }

  // sort
  std::sort(C_next[i].begin(), C_next[i].begin() + num_candidates,
            [&](Vertex* const v, Vertex* const u) {
              return D.get(i, v) + tie_breakers[v->id] <
                     D.get(i, u) + tie_breakers[u->id];
            });

  for (size_t k = 0; k < num_candidates; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != nullptr) continue;

    // avoid following conflicts
    auto& ak = occupied_now[u->id];
    if (ak != nullptr && ak != ai) {
      if (ak->v_next == nullptr) {
        // preemptively reserve current location
        occupied_next[ai->v_now->id] = ai;
        ai->v_next = ai->v_now;

        if (funcPIBT_no_following(ak, ai)) return true;

        // revert if priority inheritance failed
        occupied_next[ai->v_now->id] = nullptr;
        ai->v_next = nullptr;
      }
      continue;
    }

    // success
    occupied_next[u->id] = ai;
    ai->v_next = u;
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

Solution solve(const Instance& ins, const int verbose, const Deadline* deadline,
               std::mt19937* MT, const int threshold,
               const bool allow_following)
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tpre-processing");
  auto planner =
      Planner(&ins, deadline, MT, verbose, threshold, allow_following);
  return planner.solve();
}
