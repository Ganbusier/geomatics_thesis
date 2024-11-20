# ##########################################################
cmake_minimum_required(VERSION 3.14)
project( geomatics_thesis )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# CGAL
set(CGAL_DIR ${CMAKE_SOURCE_DIR}/3rd_party/CGAL-5.6.2)

# Easy3D
set(Easy3D_PATH ${CMAKE_SOURCE_DIR}/3rd_party/Easy3D)
set(Easy3D_BUILD_TUTORIALS OFF CACHE BOOL "Build Easy3D tutorials" FORCE)
add_subdirectory(${Easy3D_PATH})
set(Easy3D_DIR ${CMAKE_SOURCE_DIR}/build)

# Boost
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()

# PCL
find_package(PCL REQUIRED)

if ( NOT PCL_FOUND )
  message(STATUS "This project requires the PCL library, and will not be compiled.")
  return()
endif()

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# include for local directory
include_directories( ${CMAKE_SOURCE_DIR}/include )

# Creating entries for all C++ files from src directory
FILE(GLOB SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${CGAL_LIBRARIES} easy3d::viewer easy3d::algo easy3d::core)
# ##########################################################



