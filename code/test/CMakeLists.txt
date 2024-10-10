cmake_minimum_required (VERSION 3.14)
project (test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(${PROJECT_NAME}_VERSION_MAJOR 1)
set(${PROJECT_NAME}_VERSION_MINOR 0)

include_directories( ${CMAKE_SOURCE_DIR}/include/ )

find_package(PCL REQUIRED)
find_package(CGAL REQUIRED)
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt6)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CGAL REQUIRED)

include_directories(${CGAL_INCLUDE_DIRS})
link_directories(${CGAL_LIBRARY_DIRS})
add_definitions(${CGAL_DEFINITIONS})

FILE(GLOB SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})
<<<<<<< Updated upstream
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES}) 
target_link_libraries(${PROJECT_NAME} CGAL::CGAL)

if(CGAL_Qt6_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC CGAL::CGAL_Basic_viewer)
endif()
=======
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${CGAL_LIBRARIES})
>>>>>>> Stashed changes

target_compile_options(${PROJECT_NAME} PRIVATE -msse3 -msse4.1)