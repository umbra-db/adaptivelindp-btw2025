cmake_minimum_required(VERSION 3.28)
project(fastikkbz)

set(CMAKE_CXX_STANDARD 26)

add_executable(fastoptim
        src/Benchmark.cpp
        src/BumpAlloc.cpp
        src/CSV.cpp
        src/DP.cpp
        src/LinDP.cpp
        src/QueryGraph.cpp
        src/IKKBZ.cpp
        src/MDQ.cpp
        src/UnionFind.cpp
        src/Scheduler.cpp
        src/main.cpp
)
