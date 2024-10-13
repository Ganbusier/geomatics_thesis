cmake_minimum_required(VERSION 3.14)
project(test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)

include_directories(${CMAKE_SOURCE_DIR}/include/)

find_package(OpenMP REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CGAL REQUIRED)
link_directories(${CGAL_LIBRARY_DIRS})

FILE(GLOB SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} CGAL::CGAL)

if (OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()
