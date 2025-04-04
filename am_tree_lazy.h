#ifndef AM_TREE_LAZY_H_
#define AM_TREE_LAZY_H_

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

#include "incremental_mst.h"

const float alpha = 1.5;

template <AMTreeMode mode>
struct AMTreeLazy : public IncrementalMST {
  std::string Name() {
    std::string name = std::string("lazy-") + ToString(mode);
    return name;
  }

  AMTreeLazy(int n) : IncrementalMST(n) {
    p.assign(n, -1);
    t.assign(n, -1);
    s.assign(n, 1);
    idx.resize(n);
  }

#ifdef PERSISTENT
  void Insert(int u, int v, int w, int ts) {
#else
  void Insert(int u, int v, int w) {
#endif  // PERSISTENT

#ifdef PERSISTENT
    global_ts = ts;
#endif  // PERSISTENT

    if constexpr (mode == AMTreeMode::Perch) {
      LinkByPerch(w, u, v);
    } else {
      LinkByStitch(w, u, v);
    }
  }

  void LinkByPerch(int ts, int u, int v) {
    if (u == v) return;

    UpwardMaintain(u);
    UpwardMaintain(v);
    Reroot(v, -1);
    Reroot(u, v);
    assert(p[u] == -1 || p[u] == v);
    if (p[u] == v) {
      if (ts < t[u]) {
        t[u] = ts;
      }
    } else {
      if (s[u] > s[v]) std::swap(u, v);
      p[u] = v;
      t[u] = ts;
      s[v] += s[u];
    }
#ifdef PERSISTENT
    add_idx(u, global_ts, p[u], t[u]);
#endif
  }

  void LinkByStitch(int ts, int u, int v) {
    if (u == v) return;

    int x = u, y = v, max_t = std::numeric_limits<int>::min(), max_x = -1;
    for (;;) {
      if (x == y) break;
      if (s[x] > s[y]) std::swap(x, y);
      if (t[x] == -1) break;
      if (t[x] > max_t) {
        max_t = t[x];
        max_x = x;
      }
      x = p[x];
    }
    if (x == y) {
      x = max_x;
      for (int y = p[x]; y != -1; y = p[y]) {
        s[y] -= s[x];
      }
      p[x] = -1;
      t[x] = -1;
#ifdef PERSISTENT
      add_idx(x, global_ts, p[x], t[x]);
#endif
    }

    x = u, y = v;
    int w = ts, dx = 0, dy = 0;
    while (x != -1 && y != -1) {
      if (w >= t[x]) {
        s[p[x]] += dx;
        x = p[x];
      } else if (w >= t[y]) {
        s[p[y]] += dy;
        y = p[y];
      } else {
        if (s[x] > s[y]) {
          std::swap(x, y);
          std::swap(dx, dy);
        }
        dx -= s[x];
        dy += s[x];
        s[y] += s[x];
        int px_old = p[x];
        int t_old = t[x];
        p[x] = y;
        t[x] = w;
#ifdef PERSISTENT
        add_idx(x, global_ts, p[x], t[x]);
#endif
        w = t_old;
        if (px_old != -1) s[px_old] += dx;
        x = px_old;
      }
    }
    assert(x == -1 && y != -1);
    if (y != -1) {
      y = p[y];
      while (y != -1) {
        s[y] += dy;
        y = p[y];
      }
    }

    if (s[u] > s[v]) std::swap(u, v);
    for (int x = u; x != -1; x = p[x]) {
      while (p[x] != -1) {
        int y = p[x];
        if (s[x] * alpha <= s[y]) break;
        Rotate(x, y);
      }
    }
  }

#ifdef PERSISTENT
  std::optional<int> PathMax(int u, int v, int ed) {
#else
  std::optional<int> PathMax(int u, int v) {
#endif  // PERSISTENT
    if (u == v) return std::nullopt;
    static std::vector<std::pair<int, int>> a, b;
    a.clear(), b.clear();
    for (int x = u; x != -1;) {
#ifdef PERSISTENT
      auto [p, w] = get_pw(x, ed);
#else
      auto [p, w] = get_pw(x);
#endif  // PERSISTENT
      a.push_back({x, w});
      x = p;
    }
    for (int x = v; x != -1;) {
#ifdef PERSISTENT
      auto [p, w] = get_pw(x, ed);
#else
      auto [p, w] = get_pw(x);
#endif  // PERSISTENT
      b.push_back({x, w});
      x = p;
    }
    if (a.back() == b.back()) {
      int pa = a.size() - 1, pb = b.size() - 1;
      while (pa > 0 && pb > 0 && a[pa - 1] == b[pb - 1]) {
        pa--, pb--;
      }
      int ma = std::numeric_limits<int>::min();
      int mb = std::numeric_limits<int>::min();
      for (int i = 0; i < pa; i++) {
        ma = std::max(ma, a[i].second);
      }
      for (int i = 0; i < pb; i++) {
        mb = std::max(mb, b[i].second);
      }
      return std::max(ma, mb);
    } else {
      return std::nullopt;
    }
  }

 private:
  struct IEdge {
    int t, p, w;
    IEdge(int t, int p, int w) : t(t), p(p), w(w) {}
    operator std::string() {
      return "(" + std::to_string(t) + "," + std::to_string(p) + "," +
             std::to_string(w) + ")";
    }
    friend bool operator<(const int t, const IEdge& x) { return t < x.t; }
  };

  int n, global_ts;
  std::vector<int> p, t, s;
  std::vector<std::vector<IEdge>> idx;
  std::vector<int> a, b;

#ifdef PERSISTENT
  void add_idx(int u, int t, int p, int w) {
    if (!idx[u].empty() && t == idx[u].back().t) {
      idx[u].back().p = p;
      idx[u].back().w = w;
    } else if (idx[u].empty() ||
               (p != idx[u].back().p || w != idx[u].back().w)) {
      idx[u].emplace_back(t, p, w);
    }
  }
#endif  // PERSISTENT

#ifdef PERSISTENT
  std::pair<int, int> get_pw(int u, int ti = -1) {
    auto it = std::upper_bound(idx[u].begin(), idx[u].end(), ti);
    if (it == idx[u].begin()) {
      return {-1, -1};
    }
    --it;
    return {it->p, it->w};
  }
#else
  std::pair<int, int> get_pw(int u) { return {p[u], t[u]}; }
#endif  // PERSISTENT

  inline void ShortCut(int x) {
    int y = p[x];
    s[y] -= s[x];
    p[x] = p[y];
#ifdef PERSISTENT
    add_idx(x, global_ts, p[x], t[x]);
#endif
  }

  inline void ZigZag(int x) {
    int y = p[x];
    s[y] -= s[x];
    s[x] += s[y];
    std::swap(t[x], t[y]);
    p[x] = p[y];
    p[y] = x;
#ifdef PERSISTENT
    add_idx(x, global_ts, p[x], t[x]);
    add_idx(y, global_ts, p[y], t[y]);
#endif
  }

  // y = p[x]
  inline void Rotate(int x, int y) {
    if (t[x] >= t[y]) {
      ShortCut(x);
    } else {
      ZigZag(x);
    }
  }

  inline void UpwardMaintain(int x) {
    while (p[x] != -1) {
      int y = p[x];
      if (s[x] * alpha <= s[y]) {
        x = y;
      } else {
        Rotate(x, y);
      }
    }
  }

  void Reroot(int x, int watch) {
    while (p[x] != -1 && p[x] != watch) {
      Rotate(x, p[x]);
    }
  }
};

#endif  // AM_TREE_LAZY_H_
