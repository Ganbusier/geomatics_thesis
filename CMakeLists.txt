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

# CGAL
set(CGAL_DIR ${CMAKE_SOURCE_DIR}/3rd_party/CGAL-5.6.2)
find_package(CGAL REQUIRED PATHS ${CGAL_DIR} NO_DEFAULT_PATH) 
if ( NOT CGAL_FOUND )
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()
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
# include_directories( ${CMAKE_SOURCE_DIR}/include )

# Creating entries for all C++ files from src directory
FILE(GLOB SRC_FILES src/main.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}
                      PUBLIC
                      ${CGAL_LIBRARIES} 
                      easy3d::viewer 
                      easy3d::algo 
                      easy3d::core 
                      easy3d::fileio 
                      easy3d::util
                      
                      PRIVATE
                      rerun_sdk)
# ##########################################################



