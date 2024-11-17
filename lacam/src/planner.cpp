#include "../include/planner.hpp"

Constraint::Constraint()
    : who(std::vector<int>()), where(std::vector<State>()), depth(0)
{
}

Constraint::Constraint(Constraint* parent, int i, State s)
    : who(parent->who), where(parent->where), depth(parent->depth + 1)
{
  who.push_back(i);
  where.push_back(s);
}

Constraint::~Constraint(){};

Node::Node(Config _C, DistTableMultiGoal& D, Node* _parent)
    : C(_C),
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
    for (size_t i = 0; i < N; ++i) {
      priorities[i] = (float)D.get(i, C[i]) / N;
    }
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
                 std::mt19937* _MT, int _verbose, std::optional<int> _threshold,
                 bool _allow_following)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      threshold(_threshold),
      allow_following(_allow_following),
      N(ins->N),
      V_size(ins->G->size()),
      D(DistTableMultiGoal(ins)),
      tie_breakers(
          std::vector<float>(V_size * Orientation::NUM_ORIENTATIONS, 0)),
      A(Agents(N, nullptr)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr))
{
}

Solution Planner::solve()
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart search");

  // setup agents
  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

  // setup search queues
  std::stack<Node*> OPEN;
  std::unordered_map<Config, Node*, ConfigHasher> CLOSED;
  std::vector<Constraint*> GC;  // garbage collection of constraints

  // insert initial node
  auto initial_config = ins->starts;
  auto S = new Node(initial_config, D);
  OPEN.push(S);
  CLOSED[S->C] = S;

  // depth first search
  int loop_cnt = 0;
  std::vector<Config> solution;

  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;

    // do not pop here!
    S = OPEN.top();

    // check goal condition
    auto goal_reached = threshold.has_value()
                            ? S->C.enough_goals_reached(threshold.value())
                            : ins->is_goal_config(S->C);
    if (goal_reached) {
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
      auto candidates = S->C[i].get_neighbors();
      candidates.push_back(S->C[i]);
      if (MT != nullptr)
        std::shuffle(candidates.begin(), candidates.end(), *MT);  // randomize
      for (auto u : candidates) S->search_tree.push(new Constraint(M, i, u));
    }

    // create successors at the high-level search
    if (!get_new_config(S, M)) continue;

    // create new configuration
    auto C = Config(N);
    for (auto a : A) C[a->id] = a->s_next;
    ins->update_goal_indices(C, S->C);

    // check explored list
    auto iter = CLOSED.find(C);
    if (iter != CLOSED.end()) {
      OPEN.push(iter->second);
      continue;
    }

    // insert new search node
    auto S_new = new Node(C, D, S);
    OPEN.push(S_new);
    CLOSED[S_new->C] = S_new;
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
    if (a->s_now.v != nullptr && occupied_now[a->s_now.v->id] == a) {
      occupied_now[a->s_now.v->id] = nullptr;
    }
    if (a->s_next.v != nullptr) {
      occupied_next[a->s_next.v->id] = nullptr;
      a->s_next = State();
    }

    // set occupied now
    a->s_now = S->C[a->id];
    occupied_now[a->s_now.v->id] = a;
  }

  // add constraints
  for (auto k = 0; k < M->depth; ++k) {
    const auto i = M->who[k];          // agent
    const auto l = M->where[k].v->id;  // loc

    // check vertex collision
    if (occupied_next[l] != nullptr) return false;

    if (allow_following) {
      // check swap collision
      auto l_pre = S->C[i].v->id;
      if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
          occupied_next[l_pre]->id == occupied_now[l]->id) {
        return false;
      }
    } else {
      // check following conflict
      if (occupied_now[l] != nullptr && occupied_now[l] != A[i]) return false;
    }

    // set occupied_next
    A[i]->s_next = M->where[k];
    occupied_next[l] = A[i];
  }

  // perform PIBT
  for (auto k : S->order) {
    auto a = A[k];
    if (a->s_next.v == nullptr && !funcPIBT(a))
      return false;  // planning failure
  }
  return true;
}

bool Planner::funcPIBT(Agent* ai, Agent* caller)
{
  const auto i = ai->id;
  const auto neighbors = ai->s_now.get_neighbors();
  const auto K = neighbors.size();

  // get candidates for next locations
  std::vector<State> candidates;
  for (size_t k = 0; k < K; ++k) {
    auto u = neighbors[k];
    candidates.push_back(u);
    if (MT != nullptr)
      tie_breakers[u.pose_id] = get_random_float(MT);  // set tie-breaker
  }
  if (caller == nullptr) {
    candidates.push_back(ai->s_now);
  }

  std::sort(candidates.begin(), candidates.end(), [&](State s, State t) {
    return D.get(i, s) + tie_breakers[s.pose_id] <
           D.get(i, t) + tie_breakers[t.pose_id];
  });

  for (size_t k = 0; k < candidates.size(); ++k) {
    auto u = candidates[k];

    // avoid vertex conflicts
    if (occupied_next[u.v->id] != nullptr) continue;

    auto& ak = occupied_now[u.v->id];

    if (allow_following) {
      // avoid swap conflicts with constraints
      if (ak != nullptr && ak->s_next.v == ai->s_now.v) continue;

      // reserve next location
      occupied_next[u.v->id] = ai;
      ai->s_next = u;

      // empty or stay
      if (ak == nullptr || u.v == ai->s_now.v) return true;

      // priority inheritance
      if (ak->s_next.v == nullptr && !funcPIBT(ak)) continue;

      // success to plan next one step
      return true;
    } else {  // no following
      if (ak != nullptr && ak != ai) {
        if (ak->s_next.v == nullptr) {
          // preemptively reserve current location
          occupied_next[ai->s_now.v->id] = ai;
          ai->s_next = ai->s_now;

          if (funcPIBT(ak, ai)) return true;

          // revert if priority inheritance failed
          occupied_next[ai->s_now.v->id] = nullptr;
          ai->s_next = State();
        }
        continue;
      }

      // success
      occupied_next[u.v->id] = ai;
      ai->s_next = u;
      return true;
    }
  }

  // failed to secure node
  occupied_next[ai->s_now.v->id] = ai;
  ai->s_next = ai->s_now;
  return false;
}

Solution solve(const Instance& ins, const int verbose, const Deadline* deadline,
               std::mt19937* MT, const std::optional<int> threshold,
               const bool allow_following)
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tpre-processing");
  auto planner =
      Planner(&ins, deadline, MT, verbose, threshold, allow_following);
  return planner.solve();
}
