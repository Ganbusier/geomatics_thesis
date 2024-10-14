cmake_minimum_required(VERSION 3.14)
project(test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)

include_directories(${CMAKE_SOURCE_DIR}/include/)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CGAL REQUIRED)
link_directories(${CGAL_LIBRARY_DIRS})

FILE(GLOB SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})

# include(FetchContent)
# FetchContent_Declare(rerun_sdk URL
#     https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
# FetchContent_MakeAvailable(rerun_sdk)

# set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES} CGAL::CGAL)
# target_link_libraries(${PROJECT_NAME} PRIVATE rerun_cpp_sdk)