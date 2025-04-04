cmake_minimum_required(VERSION 3.18)

project(
  AM-tree
  VERSION 1.0
  DESCRIPTION ""
  HOMEPAGE_URL ""
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(
  -g
  -O3
  -march=native
)

add_executable(temporal_connectivity_streaming temporal_connectivity.cpp)

add_executable(temporal_connectivity_persistent temporal_connectivity.cpp)
target_compile_options(temporal_connectivity_persistent PRIVATE -DPERSISTENT)
