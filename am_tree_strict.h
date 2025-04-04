#ifndef AM_TREE_STRICT_H_
#define AM_TREE_STRICT_H_

#include <array>
#include <bit>
#include <cassert>
#include <cstdint>
#include <limits>
#include <vector>

#include "incremental_mst.h"

struct ChildrenSizeMaintainer {
  using WT = uint32_t;
  constexpr static int WS = 32;
  std::vector<WT> w;
  std::vector<std::array<std::vector<int>, WS>> children;
  std::vector<int> pos;

  void Init(int n) {
    w.assign(n, 0);
    children.assign(n, std::array<std::vector<int>, WS>());
    pos.assign(n, -1);
  }

  void AddChild(int u, int v, int s) {
    if (u == -1) return;
    // debug("AddChild", u, v, s);
    int ss = std::bit_width((WT)s);
    children[u][ss].push_back(v);
    if (children[u][ss].size() == 1) {
      w[u] |= (WT)1 << ss;
    }
    pos[v] = children[u][ss].size() - 1;
  }

  void DelChild(int u, int v, int s) {
    if (u == -1) return;
    // debug("DelChild", u, v, s);
    int ss = std::bit_width((WT)s);
    assert(children[u][ss][pos[v]] == v);
    if (pos[v] + 1 < children[u][ss].size()) {
      children[u][ss][pos[v]] = children[u][ss].back();
      pos[children[u][ss].back()] = pos[v];
    }
    pos[v] = -1;
    children[u][ss].pop_back();
    if (children[u][ss].empty()) {
      w[u] &= ~((WT)1 << ss);
    }
  }

  int CheckMax(int u) {
    int hi = WS - 1 - std::countl_zero(w[u]);
    if (hi < 0) return -1;
    if (children[u][hi].size() > 1) return -1;
    return children[u][hi][0];
  }
};

template <AMTreeMode mode>
struct AMTreeStrict : public IncrementalMST {
  std::string Name() {
    std::string name = std::string("strict-") + ToString(mode);
    return name;
  }

  AMTreeStrict(int n) : IncrementalMST(n) {
    p.assign(n, -1);
    t.assign(n, -1);
    s.assign(n, 1);
    idx.resize(n);
    maintainer.Init(n);
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
      maintainer.AddChild(v, u, s[u]);
    }
#ifdef PERSISTENT
    add_idx(u, global_ts, p[u], t[u]);
#endif  // PERSISTENT

    for (int x = u; x != -1; x = p[x]) {
      DownwardMaintain(x);
    }
  }

  void LinkByStitch(int ts, int u, int v) {
    if (u == v) return;

    a.clear();
    for (int x = u; x != -1; x = p[x]) a.push_back(x);
    for (int x = v; x != -1; x = p[x]) a.push_back(x);

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
        maintainer.DelChild(p[y], y, s[y]);
        s[y] -= s[x];
        maintainer.AddChild(p[y], y, s[y]);
      }
      maintainer.DelChild(p[x], x, s[x]);
      p[x] = -1;
      t[x] = -1;
#ifdef PERSISTENT
      add_idx(x, global_ts, p[x], t[x]);
#endif  // PERSISTENT
    }

    x = u, y = v;
    int w = ts, dx = 0, dy = 0;
    while (x != -1 && y != -1) {
      if (w >= t[x]) {
        x = p[x];
        maintainer.DelChild(p[x], x, s[x]);
        s[x] += dx;
        maintainer.AddChild(p[x], x, s[x]);
      } else if (w >= t[y]) {
        y = p[y];
        maintainer.DelChild(p[y], y, s[y]);
        s[y] += dy;
        maintainer.AddChild(p[y], y, s[y]);
      } else {
        if (s[x] > s[y]) {
          std::swap(x, y);
          std::swap(dx, dy);
        }
        dx -= s[x];
        dy += s[x];
        int px_old = p[x];
        int t_old = t[x];
        maintainer.DelChild(px_old, x, s[x]);
        maintainer.AddChild(y, x, s[x]);

        maintainer.DelChild(p[y], y, s[y]);
        s[y] += s[x];
        maintainer.AddChild(p[y], y, s[y]);

        p[x] = y;
        t[x] = w;
#ifdef PERSISTENT
        add_idx(x, global_ts, p[x], t[x]);
#endif  // PERSISTENT
        w = t_old;

        x = px_old;
        if (px_old != -1) {
          maintainer.DelChild(p[x], x, s[x]);
          s[x] += dx;
          maintainer.AddChild(p[x], x, s[x]);
        }

        if (px_old != -1) a.push_back(px_old);
      }
    }
    assert(x == -1 && y != -1);
    if (y != -1) {
      y = p[y];
      while (y != -1) {
        maintainer.DelChild(p[y], y, s[y]);
        s[y] += dy;
        maintainer.AddChild(p[y], y, s[y]);
        y = p[y];
      }
    }

    for (int x : a) {
      DownwardMaintain(x);
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
  ChildrenSizeMaintainer maintainer;

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
    int y = p[x], z = p[y];
    maintainer.DelChild(z, y, s[y]);
    s[y] -= s[x];
    maintainer.AddChild(z, y, s[y]);
    p[x] = p[y];
    maintainer.DelChild(y, x, s[x]);
    maintainer.AddChild(z, x, s[x]);
#ifdef PERSISTENT
    add_idx(x, global_ts, p[x], t[x]);
#endif  // PERSISTENT
  }

  inline void ZigZag(int x) {
    int y = p[x], z = p[y];
    maintainer.DelChild(z, y, s[y]);
    maintainer.DelChild(y, x, s[x]);
    maintainer.AddChild(z, x, s[y]);
    s[y] -= s[x];
    s[x] += s[y];
    maintainer.AddChild(x, y, s[y]);
    std::swap(t[x], t[y]);
    p[x] = z;
    p[y] = x;
#ifdef PERSISTENT
    add_idx(x, global_ts, p[x], t[x]);
    add_idx(y, global_ts, p[y], t[y]);
#endif  // PERSISTENT
  }

  // y=p[x]
  inline void Rotate(int x, int y) {
    if (t[x] >= t[y]) {
      ShortCut(x);
    } else {
      ZigZag(x);
    }
  }

  void DownwardMaintain(int x) {
    for (;;) {
      int y = maintainer.CheckMax(x);
      if (y == -1) break;
      if (s[y] * 1.5 <= s[x]) break;
      Rotate(y, x);
#ifdef PERSISTENT
      add_idx(y, global_ts, p[y], t[y]);
#endif  // PERSISTENT
    }
  }

  void Reroot(int x, int watch) {
    while (p[x] != -1 && p[x] != watch) {
      int z = p[x];
      Rotate(x, z);
      DownwardMaintain(z);
    }
  }
};

#endif  // AM_TREE_STRICT_H_
