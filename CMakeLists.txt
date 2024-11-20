# ##########################################################
cmake_minimum_required(VERSION 3.14)
project( geomatics_thesis )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CGAL_DIR ${CMAKE_SOURCE_DIR}/3rd_party/CGAL-5.6.2)

# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()

# PCL and its components
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
target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES} ${PCL_LIBRARIES})
# ##########################################################



