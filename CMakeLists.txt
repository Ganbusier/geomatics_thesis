# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.14)
project( geomatics_thesis )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CGAL_SOURCE_DIR ${CMAKE_SOURCE_DIR}/3rd_party/CGAL-5.6.2)

# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()

endif()

# include for local directory
include_directories( ${CGAL_SOURCE_DIR}/include )
include_directories( ${CMAKE_SOURCE_DIR}/include )
add_subdirectory(${CGAL_SOURCE_DIR})

# include for local package
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Creating entries for all C++ files from src directory
FILE(GLOB SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES} ${PCL_LIBRARIES})
# ##########################################################



