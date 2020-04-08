cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall -g")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

find_program(CMAKE_CXX_CPPCHECK NAMES cppcheck)


list(
    APPEND CMAKE_CXX_CPPCHECK 
        "--enable=warning"
        "--inconclusive"
        "--force" 
        "--inline-suppr"
        "--suppressions-list=${CMAKE_SOURCE_DIR}/CppCheckSuppressions.txt"
)



add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp src/ransac.cpp src/cluster.cpp )
target_link_libraries (environment ${PCL_LIBRARIES})
