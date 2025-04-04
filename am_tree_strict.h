#ifndef TYPE2_DYNAMIC_CONNECTIVITY_SIZE_BALANCED_PREEMTIVE_H_
#define TYPE2_DYNAMIC_CONNECTIVITY_SIZE_BALANCED_PREEMTIVE_H_

#include <bit>

#include "debug.h"
#include "pam/pam.h"
#include "type2_dynamic_connectivity.h"

template <typename T = int>
struct ChildrenSizeMaintainer {
  std::vector<WT> w;
  std::vector<std::array<std::vector<T>, WS>> children;
  std::vector<T> pos;

  void Init(T n) {
    w.assign(n, 0);
    children.assign(n, std::array<std::vector<T>, WS>());
    pos.assign(n, -1);
  }

  void AddChild(T u, T v, T s) {
    if (u == -1) return;
    // debug("AddChild", u, v, s);
    T ss = std::bit_width((WT)s);
    children[u][ss].push_back(v);
    if (children[u][ss].size() == 1) {
      w[u] |= (WT)1 << ss;
    }
    pos[v] = children[u][ss].size() - 1;
  }

  void DelChild(T u, T v, T s) {
    if (u == -1) return;
    // debug("DelChild", u, v, s);
    T ss = std::bit_width((WT)s);
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

  T CheckMax(T u) {
    T hi = WS - 1 - std::countl_zero(w[u]);
    if (hi < 0) return -1;
    if (children[u][hi].size() > 1) return -1;
    return children[u][hi][0];
  }
};

template <typename T = int, short CONFIG = 0, bool MAINTAIN_EDGE_SET = false>
struct PreemptiveSizeBalancedType2DynamicConnectivity
    : public Type2DynamicConnectivity<T> {
  std::string Name() {
    std::string name = "preemptive-size-balanced-" + std::to_string(CONFIG);
    if (MAINTAIN_EDGE_SET) name += "-count-cc";
    else name += "-normal";
    return name;
  }

  void Init(T n) {
    this->n = n;
    p.assign(n, -1);
    t.assign(n, -1);
    s.assign(n, 1);
    idx.resize(n);
    maintainer.Init(n);
  }

  void Add(T ts, T u, T v) {
    // debug("Add", ts, u, v);
    global_ts = ts;
    if constexpr (CONFIG == 0) {
      Add0(ts, u, v);
    } else {
      Add1(ts, u, v);
    }
    // debug(p, t);
  }

  void Add1(T ts, T u, T v) {
    if (u == v) return;

#ifdef STREAMING
#else
    global_ts = ts;
#endif
    Reroot(v, -1);
    Reroot(u, v);
    assert(p[u] == -1 || p[u] == v);
    if (p[u] == v) {
      if (ts > t[u]) {
        if constexpr (MAINTAIN_EDGE_SET) {
          edge_change_list.push_back({ts, {t[u], -1}});
        }
        t[u] = ts;
        if constexpr (MAINTAIN_EDGE_SET) {
          edge_change_list.push_back({ts, {ts, 1}});
        }
      }
    } else {
      if (s[u] > s[v]) std::swap(u, v);
      p[u] = v;
      t[u] = ts;
      s[v] += s[u];
      maintainer.AddChild(v, u, s[u]);
      if constexpr (MAINTAIN_EDGE_SET) {
        edge_change_list.push_back({ts, {ts, 1}});
      }
    }
#ifdef STREAMING
#else
    add_idx(u, ts, p[u], t[u]);
#endif

    for (T x = u; x != -1; x = p[x]) {
      DownwardMaintain(x);
    }

    // CheckStatus(true);
  }

  void Add0(T ts, T u, T v) {
    if (u == v) return;

    a.clear();
    for (T x = u; x != -1; x = p[x]) a.push_back(x);
    for (T x = v; x != -1; x = p[x]) a.push_back(x);

    T x = u, y = v, min_t = std::numeric_limits<T>::max(), min_x = -1;
    for (;;) {
      if (x == y) break;
      if (s[x] > s[y]) std::swap(x, y);
      if (t[x] == -1) break;
      if (t[x] < min_t) {
        min_t = t[x];
        min_x = x;
      }
      x = p[x];
    }
    if (x == y) {
      x = min_x;
      for (T y = p[x]; y != -1; y = p[y]) {
        maintainer.DelChild(p[y], y, s[y]);
        s[y] -= s[x];
        maintainer.AddChild(p[y], y, s[y]);
      }
      if constexpr (MAINTAIN_EDGE_SET) {
        edge_change_list.push_back({ts, {t[x], -1}});
      }
      maintainer.DelChild(p[x], x, s[x]);
      p[x] = -1;
      t[x] = -1;
#ifdef STREAMING
#else
      add_idx(x, ts, p[x], t[x]);
#endif
    }

    if constexpr (MAINTAIN_EDGE_SET) {
      edge_change_list.push_back({ts, {ts, -1}});
    }

    // CheckStatus(false);

    // T x = u, y = v;
    x = u, y = v;
    T w = ts, dx = 0, dy = 0;
    while (x != -1 && y != -1) {
      if (w <= t[x]) {
        x = p[x];
        maintainer.DelChild(p[x], x, s[x]);
        s[x] += dx;
        maintainer.AddChild(p[x], x, s[x]);
      } else if (w <= t[y]) {
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
        T px_old = p[x];
        T t_old = t[x];
        maintainer.DelChild(px_old, x, s[x]);
        maintainer.AddChild(y, x, s[x]);

        maintainer.DelChild(p[y], y, s[y]);
        s[y] += s[x];
        maintainer.AddChild(p[y], y, s[y]);

        p[x] = y;
        t[x] = w;
#ifdef STREAMING
#else
        add_idx(x, ts, p[x], t[x]);
#endif
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

    for (T x : a) {
      DownwardMaintain(x);
    }

    // CheckStatus(true);
  }

  T Query(T ed, T u, T v) {
    assert(u != v);
    static std::vector<std::pair<T, T>> a, b;
    a.clear(), b.clear();
    for (T x = u; x != -1;) {
      auto [p, w] = get_pw(x, ed);
      a.push_back({x, w});
      x = p;
    }
    for (T x = v; x != -1;) {
      auto [p, w] = get_pw(x, ed);
      b.push_back({x, w});
      x = p;
    }
    if (a.back() == b.back()) {
      T pa = a.size() - 1, pb = b.size() - 1;
      while (pa > 0 && pb > 0 && a[pa - 1] == b[pb - 1]) {
        pa--, pb--;
      }
      T ma = std::numeric_limits<T>::max();
      T mb = std::numeric_limits<T>::max();
      for (T i = 0; i < pa; i++) {
        ma = std::min(ma, a[i].second);
      }
      for (T i = 0; i < pb; i++) {
        mb = std::min(mb, b[i].second);
      }
      return std::min(ma, mb);
    } else {
      return -1;
    }
  }

  T QueryOnline(T u, T v) { return Query(std::numeric_limits<T>::max(), u, v); }

  void PrintStatus() {
    CheckStatus(true);
    T max_h = 0;
    long long sum_h = 0;
    for (T i = 0; i < n; i++) {
      T h = 0;
      for (T x = i; x != -1; x = p[x]) h++;
      if (h > max_h) max_h = h;
      sum_h += h;
    }
    double avg_h = double(sum_h) / n;
    double avg_d = DP(n, p).second;
    debug(max_h, avg_h, avg_d);
    long long sum_len = 0;
    for (T i = 0; i < n; i++) {
      sum_len += idx[i].size();
    }
    double avg_len = double(sum_len) / n;
    debug(avg_len);
  }

  void PostBuild() {
    if constexpr (!MAINTAIN_EDGE_SET) return;
    std::cout << "PostBuild start " << edge_change_list.size() << std::endl;
#ifdef STREAMING
    map_t map;
    map = map_t::multi_insert_combine(
        map,
        parlay::tabulate(edge_change_list.size(),
                         [&](size_t i) { return edge_change_list[i].second; }),
        std::plus<T>());
    rec.resize(1);
    rec[0] = map;
#else
    size_t tot = edge_change_list.size();
    size_t block_size = std::max(1.0, sqrt(tot));
    size_t num_blocks = tot / block_size + (tot % block_size == 0 ? 0 : 1);
    map_t map;
    parlay::sequence<map_t> B(num_blocks);
    debug(tot, block_size, num_blocks);
    for (size_t i = 0; i < num_blocks; i++) {
      size_t l = i * block_size;
      size_t r = std::min(tot - 1, l + block_size - 1);
      map = map_t::multi_insert_combine(
          map,
          parlay::tabulate(
              r - l + 1,
              [&](size_t j) { return edge_change_list[l + j].second; }),
          std::plus<T>());
      B[i] = map;
    }
    rec.resize(tot);
    parlay::parallel_for(0, num_blocks, [&](size_t i) {
      size_t l = i * block_size;
      size_t r = std::min(tot - 1, l + block_size - 1);
      map_t map;
      if (i > 0) map = B[i - 1];
      for (size_t j = l; j <= r; j++) {
        map = map_t::multi_insert_combine(
            map,
            parlay::tabulate(
                1, [&](size_t j) { return edge_change_list[l + j].second; }),
            std::plus<T>());
        rec[j] = map;
      }
    });
#endif
  }

 private:
  struct entry {
    using key_t = T;
    using val_t = T;
    using aug_t = size_t;
    static inline bool comp(key_t a, key_t b) { return a < b; }
    static aug_t get_empty() { return 0; }
    static aug_t from_entry(key_t k, val_t v) { return v; }
    static aug_t combine(aug_t a, aug_t b) { return a + b; }
  };
  using map_t = aug_map<entry>;
  parlay::sequence<map_t> rec;
  std::vector<std::pair<T, std::pair<T, T>>> edge_change_list;

  struct IEdge {
    T t, p, w;
    IEdge(T t, T p, T w) : t(t), p(p), w(w) {}
    operator std::string() {
      return "(" + std::to_string(t) + "," + std::to_string(p) + "," +
             std::to_string(w) + ")";
    }
    friend bool operator<(const T t, const IEdge& x) { return t < x.t; }
  };

  T n, global_ts;
  std::vector<T> p, t, s;
  std::vector<std::vector<IEdge>> idx;
  std::vector<T> a, b;
  ChildrenSizeMaintainer<T> maintainer;

  void add_idx(T u, T t, T p, T w) {
#ifdef STREAMING
    return;
#else
    if (!idx[u].empty() && t == idx[u].back().t) {
      idx[u].back().p = p;
      idx[u].back().w = w;
    } else if (idx[u].empty() ||
               (p != idx[u].back().p || w != idx[u].back().w)) {
      idx[u].emplace_back(t, p, w);
    }
#endif  // STREAMING
  }

  std::pair<T, T> get_pw(T u, T ti) {
#ifdef STREAMING
    return {p[u], t[u]};
#else
    auto it = std::upper_bound(idx[u].begin(), idx[u].end(), ti);
    if (it == idx[u].begin()) {
      return {-1, -1};
    }
    --it;
    return {it->p, it->w};
#endif  // STREAMING
  }

  inline void ShortCut(T x) {
    T y = p[x], z = p[y];
    maintainer.DelChild(z, y, s[y]);
    s[y] -= s[x];
    maintainer.AddChild(z, y, s[y]);
    p[x] = p[y];
    maintainer.DelChild(y, x, s[x]);
    maintainer.AddChild(z, x, s[x]);
#ifdef STREAMING
#else
    add_idx(x, global_ts, p[x], t[x]);
#endif
  }

  inline void ZigZag(T x) {
    T y = p[x], z = p[y];
    maintainer.DelChild(z, y, s[y]);
    maintainer.DelChild(y, x, s[x]);
    maintainer.AddChild(z, x, s[y]);
    s[y] -= s[x];
    s[x] += s[y];
    maintainer.AddChild(x, y, s[y]);
    std::swap(t[x], t[y]);
    p[x] = z;
    p[y] = x;
#ifdef STREAMING
#else
    add_idx(x, global_ts, p[x], t[x]);
    add_idx(y, global_ts, p[y], t[y]);
#endif
  }

  // y=p[x]
  inline void Rotate(T x, T y) {
    if (t[x] <= t[y]) {
      ShortCut(x);
    } else {
      ZigZag(x);
    }
  }

  void DownwardMaintain(T x) {
    for (;;) {
      int y = maintainer.CheckMax(x);
      if (y == -1) break;
      if (s[y] * 1.5 <= s[x]) break;
      Rotate(y, x);
#ifdef STREAMING
#else
      add_idx(y, global_ts, p[y], t[y]);
#endif
    }
  }

  void Reroot(T x, T watch) {
    while (p[x] != -1 && p[x] != watch) {
      T z = p[x];
      Rotate(x, z);
      DownwardMaintain(z);
    }
  }

  void CheckStatus(bool check_balance = true) {
    std::vector<int> tot(n, 1);
    for (int i = 0; i < n; i++) {
      if (p[i] != -1) {
        tot[p[i]] += s[i];
        T ss = std::bit_width((WT)s[i]);
        assert(maintainer.children[p[i]][ss][maintainer.pos[i]] == i);
      }
    }
    for (int i = 0; i < n; i++) {
      assert(s[i] == tot[i]);
    }
    if (check_balance) {
      for (int i = 0; i < n; i++) {
        int j = maintainer.CheckMax(i);
        if (j != -1) {
          if (s[j] * 1.5 > s[i]) debug("unbalance", i, j);
          assert(s[j] * 1.5 <= s[i]);
        }
      }
    }
    debug("status good!");
    if (check_balance) debug("balance good!");
  }
};

#endif  // TYPE2_DYNAMIC_CONNECTIVITY_SIZE_BALANCED_PREEMTIVE_H_
