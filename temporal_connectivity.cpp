#include <algorithm>
#include <array>
#include <cassert>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include "am_tree_lazy.h"
#include "incremental_mst.h"

using namespace std;

const int test_n = 1000;
const int test_m = 10000;
const int test_q = 1000;

template <typename T = int>
struct Dsu {
  static_assert(is_signed<T>::value);
  vector<T> fa;
  T n, cnt;
  void Init(T n) {
    this->n = n;
    fa.resize(n);
    for (T i = 0; i < n; i++) fa[i] = -1;
    cnt = n;
  }
  T Find(T x) {
    assert(0 <= x && x < n);
    if (fa[x] < 0) return x;
    return fa[x] = Find(fa[x]);
  }
  void Unite(T x, T y) {
    assert(0 <= x && x < n && 0 <= y && y < n);
    x = Find(x);
    y = Find(y);
    if (x == y) return;
    if (fa[x] < fa[y]) swap(x, y);
    fa[y] += fa[x];
    fa[x] = y;
    cnt--;
  }
};

struct BruteForceIncrementalMST : public IncrementalMST {
  BruteForceIncrementalMST(int n) : IncrementalMST(n) {}
  string Name() { return "brute-force"; }

#ifdef PERSISTENT
  void Insert(int u, int v, int w, int ts) { edges.push_back({u, v, w, ts}); }
#else
  void Insert(int u, int v, int w) { edges.push_back({u, v, w, -1}); }
#endif  // PERSISTENT

#ifdef PERSISTENT
  optional<int> PathMax(int u, int v, int ts) {
#else
  optional<int> PathMax(int u, int v) {
#endif  // PERSISTENT
    if (u == v) return nullopt;
    dsu.Init(n);
    sort(edges.begin(), edges.end(),
         [&](auto& a, auto& b) { return a[2] < b[2]; });
    vector go(n, vector<pair<int, int>>());
    for (auto [u, v, w, t] : edges) {
#ifdef PERSISTENT
      if (t > ts) continue;
#endif  // PERSISTENT
      if (dsu.Find(u) != dsu.Find(v)) {
        dsu.Unite(u, v);
        go[u].push_back({v, w});
        go[v].push_back({u, w});
      }
    }
    if (dsu.Find(u) != dsu.Find(v)) return nullopt;
    vector<bool> visit(n);
    vector<int> dist(n);
    vector<int> que;
    visit[u] = true;
    dist[u] = numeric_limits<int>::min();
    que.push_back(u);
    for (int i = 0; i < (int)que.size(); i++) {
      int x = que[i];
      for (auto [y, t] : go[x]) {
        if (!visit[y]) {
          visit[y] = true;
          dist[y] = max(dist[x], t);
          que.push_back(y);
        }
      }
    }
    return dist[v];
  }

 private:
  vector<array<int, 4>> edges;
  Dsu<int> dsu;
};

auto MakeData(int n, int m, int q) {
  vector<pair<int, pair<int, int>>> edges(m);
  mt19937 rng(0);
  for (auto& e : edges) {
    e.first = rng() % 1000000000;
    e.second.first = rng() % n;
    e.second.second = rng() % n;
  }
  sort(edges.begin(), edges.end());
  vector<array<int, 4>> queries(q);
  for (size_t i = 0; i < q; i++) {
    int st = edges[rng() % edges.size()].first;
    int ed = edges[rng() % edges.size()].first;
    if (st > ed) swap(st, ed);
    int u = rng() % n;
    int v = rng() % n;
    if (u == v) v = (v + 1) % n;
    queries[i] = {st, ed, u, v};
  }
  return make_tuple(edges, queries);
}

int main(int argc, char** argv) {
  int n = test_n;
  int q = test_q;
  auto [edges, queries] = MakeData(test_n, test_m, test_q);

  vector<IncrementalMST*> solvers = {
      new BruteForceIncrementalMST(n),
      new AMTreeLazy<AMTreeMode::Perch>(n),
      new AMTreeLazy<AMTreeMode::Stitch>(n),
  };

  vector<vector<optional<int>>> res;

  for (auto solver : solvers) {
    cout << "\n" << solver->Name() << " start" << endl;
    res.push_back(vector<optional<int>>(q));
    for (size_t i = 0; i < edges.size(); i++) {
      auto& [t, e] = edges[i];
#ifdef PERSISTENT
      solver->Insert(e.first, e.second, -t, t);
#else
      solver->Insert(e.first, e.second, -t);
#endif  // PERSISTENT
    }
    for (size_t i = 0; i < q; i++) {
      auto [st, ed, u, v] = queries[i];
#ifdef PERSISTENT
      res.back()[i] = solver->PathMax(u, v, ed);
#else
      res.back()[i] = solver->PathMax(u, v);
#endif  // PERSISTENT
    }
    cout << solver->Name() << " end" << endl;
  }
  cout << endl;

  // for (int i = 0; i < (int)res.size(); i++) {
  //   cout << solvers[i]->Name() << ": ";
  //   for (int j = 0; j < (int)res[i].size(); j++) {
  //     cout << res[i][j].value_or(-1) << " ";
  //   }
  //   cout << endl;
  // }

  if (res.size() > 1) {
    bool ok = true;
    for (int i = 1; i < (int)res.size(); i++) {
      if (res[i] != res[0]) {
        ok = false;
      }
    }
    cout << "\nok: " << (ok ? "true" : "false") << endl;
  }

  return 0;
}