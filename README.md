## The Anti-Monopoly Tree (AM-tree)

This repo contains code for our paper "New Algorithms for Incremental Minimum Spanning Trees and Temporal Graph Applications".

The **Anti-Monopoly Tree (AM-tree)** is an efficient data structure for minimum spanning tree. It supports the following operations in $O(\log n)$ amortized time:
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
