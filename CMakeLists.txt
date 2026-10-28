cmake_minimum_required(VERSION 3.16)
project(lidarsim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)

add_library(lidarsim
  src/geometry.cpp
  src/mesh.cpp
  src/bvh.cpp
  src/scene.cpp
  src/lidar.cpp
  src/io.cpp)
target_include_directories(lidarsim PUBLIC include)
target_link_libraries(lidarsim PUBLIC Threads::Threads)
target_compile_options(lidarsim PUBLIC -Wall -Wextra)

add_executable(lidarsim_cli apps/cli.cpp)
target_link_libraries(lidarsim_cli PRIVATE lidarsim)

add_executable(lidarsim_bench apps/bench.cpp)
target_link_libraries(lidarsim_bench PRIVATE lidarsim)

add_executable(lidarsim_tests
  tests/test_main.cpp
  tests/test_geometry.cpp
  tests/test_bvh.cpp
  tests/test_lidar.cpp
  tests/test_fog.cpp)
target_link_libraries(lidarsim_tests PRIVATE lidarsim)
