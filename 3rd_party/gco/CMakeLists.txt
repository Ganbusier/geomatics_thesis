cmake_minimum_required(VERSION 3.14)

set(lib_srcs
    ./GCoptimization.cpp
    ./GCoptimization.h
    ./LinkedBlockList.cpp
    ./LinkedBlockList.h
    ./block.h
    ./energy.h
    ./graph.h
    )

add_library(gco ${lib_srcs})


