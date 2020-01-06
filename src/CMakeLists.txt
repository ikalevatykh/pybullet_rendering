# Copyright (c) 2019-2020 INRIA.
# This source code is licensed under the LGPLv3 license found in the
# LICENSE file in the root directory of this source tree.

cmake_minimum_required(VERSION 3.1)
project(pybullet_rendering)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose build, options are: None Debug Release RelWithDebInfo MinSizeRel"
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# build options
option(BUILD_TEST "Build test binaries" OFF)

# dependencies
include(deps/deps.cmake)

# include source dirs
include_directories(${PROJECT_SOURCE_DIR})

# modules
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/scene")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/render")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/plugin")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/bindings")

# build tests
if (BUILD_TEST)
  # message("Building TESTS")
  # add_subdirectory(tests)
endif()
