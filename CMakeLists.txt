# ##########################################################
cmake_minimum_required(VERSION 3.14)
project( geomatics_thesis )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Boost
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()

# OSQP for CGAL shape regularization
find_package(OSQP REQUIRED)

if ( NOT OSQP_FOUND )
  message(STATUS "This project requires the OSQP library, and will not be compiled.")
  return()
endif()
set(OSQP_INCLUDE_DIR "C:/Program Files (x86)/osqp/include")
set(OSQP_LIBRARIES "C:/Program Files (x86)/osqp/lib/osqp.lib")
message(STATUS "OSQP_INCLUDE_DIR: ${OSQP_INCLUDE_DIR}")
message(STATUS "OSQP_LIBRARY: ${OSQP_LIBRARIES}")


# CGAL
set(CGAL_DIR ${CMAKE_SOURCE_DIR}/3rd_party/CGAL-6.0.1)
find_package(CGAL REQUIRED PATHS ${CGAL_DIR} NO_DEFAULT_PATH) 
if ( NOT CGAL_FOUND )
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()
endif()

# Import CGAL::OSQP_support
include("${CGAL_DIR}/cmake/modules/CGAL_OSQP_support.cmake")
if (NOT TARGET CGAL::OSQP_support)
    message(FATAL_ERROR "Failed to load CGAL::OSQP_support.")
endif()

# Easy3D
set(Easy3D_PATH ${CMAKE_SOURCE_DIR}/3rd_party/Easy3D)
set(Easy3D_BUILD_TUTORIALS OFF CACHE BOOL "Build Easy3D tutorials" FORCE)
add_subdirectory(${Easy3D_PATH})
set(Easy3D_DIR ${CMAKE_SOURCE_DIR}/build)

# rerun
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# include for local directory
include_directories( ${CMAKE_SOURCE_DIR}/include )

# Creating entries for all C++ files from src directory
FILE(GLOB SRC_FILES src/main.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}
                      PUBLIC
                      CGAL::CGAL 
                      CGAL::OSQP_support
                      easy3d::viewer 
                      easy3d::algo 
                      easy3d::core 
                      easy3d::fileio 
                      easy3d::util
                      
                      PRIVATE
                      rerun_sdk)
# ##########################################################



