#ifndef INCREMENTAL_MST_H_
#define INCREMENTAL_MST_H_

#include <optional>
#include <string>

struct IncrementalMST {
  int n;
  IncrementalMST(int n) : n(n) {}
  virtual std::string Name() = 0;
#ifdef PERSISTENT
  virtual void Insert(int u, int v, int w, int ts) = 0;
  virtual std::optional<int> PathMax(int u, int v, int ts) = 0;
#else
  virtual void Insert(int u, int v, int w) = 0;
  virtual std::optional<int> PathMax(int u, int v) = 0;
#endif  // PERSISTENT
};

enum AMTreeMode {
  Perch,
  Stitch,
};

constexpr const char* ToString(AMTreeMode c) {
  switch (c) {
    case AMTreeMode::Perch:
      return "perch";
    case AMTreeMode::Stitch:
      return "stitch";
  }
}

#endif  // INCREMENTAL_MST_H_
