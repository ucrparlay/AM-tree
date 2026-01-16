## The Anti-Monopoly Tree (AM-tree)

The **Anti-Monopoly Tree (AM-tree)** is an efficient data structure to maintain **incremental minimum spanning trees (Incremental MST)**. It supports the following operations in $O(\log n)$ amortized time:
- `void Insert(int u, int v, int w)`
- `std::optional<int> PathMax(int u, int v)`

Persistent AM-tree supports the following operations:
- `void Insert(int u, int v, int w, int ts)`: Insert an edge with timestamp `ts`. The give `ts` must be non-decreasing.
- `std::optional<int> PathMax(int u, int v, int ts)`: Query the path-max at time `ts`.

## Requirements

- CMake >= 3.18
- C++ compiler with C++20 features support

## Usage

Download code:
```
git clone --recurse-submodules git@github.com:ucrparlay/AM-tree.git
```

Compile:
```
mkdir build && cd build
cmake ..
make
```

To run the code, simply:
```shell
./build/temporal_connectivity_streaming  # offline queries
./build/temporal_connectivity_persistent  # historical queries using persistent AM-tree
```

## Citation

```
@inproceedings{ding2025new,
  title={New Algorithms for Incremental Minimum Spanning Trees and Temporal Graph Applications},
  author={Ding, Xiangyun and Gu, Yan and Sun, Yihan},
  booktitle={2025 Proceedings of the Conference on Applied and Computational Discrete Algorithms (ACDA)},
  pages={293--307},
  year={2025},
  organization={SIAM}
}
```
