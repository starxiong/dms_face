cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#--------------------------------------------
# Project setting
#--------------------------------------------
project(face_pose)
# INCLUDE(cmake/version.cmake)

MESSAGE("---------------OPTIONS-----------------")
set(TARGET "STATIC" CACHE STRING "STATIC or SHARED")
# global root dir
set(SOLUTION_DIR ${CMAKE_CURRENT_SOURCE_DIR})

option(ROS "option for debug" OFF)

# dir for common cmake files
list(APPEND CMAKE_MODULE_PATH ${SOLUTION_DIR}/cmake)
list(APPEND CMAKE_PREFIX_PATH ${SOLUTION_DIR}/cmake)

# Option for platform
set(PLATFORM "auto" CACHE STRING "auto, x86 or x64")
# Build type, configuration
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Debug or Release")
message(STATUS "CMAKE_BUILD_TYPE: "${CMAKE_BUILD_TYPE})
if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CONFIGURATION "Release")
else ()
    set(CONFIGURATION ${CMAKE_BUILD_TYPE})
endif ()

include_directories(${Pangolin_INCLUDE_DIRS})
link_directories(${Pangolin_LIBRARY_DIRS})

INCLUDE(cmake/load_packages.cmake)
INCLUDE(cmake/source.cmake)
INCLUDE(cmake/build.cmake)



