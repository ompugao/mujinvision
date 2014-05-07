# -*- coding: utf-8 -*-
# Copyright (C) 2012-2013 MUJIN Inc. <rosen.diankov@mujin.co.jp>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required (VERSION 2.8.0)
project(mujinvision)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

# Define here the needed parameters
# make sure to change the version in docs/Makefile
set (MUJINVISION_VERSION_MAJOR 0)
set (MUJINVISION_VERSION_MINOR 1)
set (MUJINVISION_VERSION_PATCH 0)
set (MUJINVISION_VERSION ${MUJINVISION_VERSION_MAJOR}.${MUJINVISION_VERSION_MINOR}.${MUJINVISION_VERSION_PATCH})
set (MUJINVISION_SOVERSION ${MUJINVISION_VERSION_MAJOR}.${MUJINVISION_VERSION_MINOR})
set (VISION_SOVERSION ${MUJINVISION_VERSION_MAJOR}.${MUJINVISION_VERSION_MINOR})
message(STATUS "Compiling MUJIN Vision Library Version ${MUJINVISION_VERSION}, soversion=${VISION_SOVERSION}")

set(MUJINVISION_CMAKE_INSTALL_DIR "mujinvision-${MUJINVISION_VERSION_MAJOR}.${MUJINVISION_VERSION_MINOR}" CACHE STRING "Directory to install the cmake config files.")
set(MUJINVISION_TARGET_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE STRING "The target processor architecture to build for, this is combined with the generator toolchain")

message(STATUS "Using cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" )
# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0002
cmake_policy(SET CMP0002 NEW)
# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0003
cmake_policy(SET CMP0003 NEW)

option(OPT_SAMPLES "Build the samples" ON)
option(OPT_BUILD_TESTS "Build the tests" OFF)
option(OPT_BUILD_STATIC "Build static libraries for the library" ON)

# Use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# When building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(LIB_SUFFIX CACHE STRING "suffix for the library directory need for x86-64 systems that use lib64 ")

# The RPATH to be used when installing
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}")

# Add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckLibraryExists)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)
include(CheckCXXSourceRuns)
include(CheckCXXCompilerFlag)

# have to include before boost since the boost headers can be located in a previously installed version of this library
set(MUJINVISION_INCLUDE_LOCAL_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(${MUJINVISION_INCLUDE_LOCAL_DIRS})

if( MSVC )
  message(STATUS "TODO: preparation for MSVC")
else()
  set(MUJINVISION_LIBRARY_SUFFIX "${MUJINVISION_SOVERSION}" CACHE STRING "Suffix to append to library names")
endif()

# MujinControllerClient
find_package(MujinControllerClient REQUIRED)
message(STATUS "Found MUJIN Controller Client C++ ${MujinControllerClient_VERSION}, with Boost ${MujinControllerClient_Boost_VERSION} (include: ${MujinControllerClient_INCLUDE_DIRS}, lib: ${MujinControllerClient_LIBRARY_DIRS})")
find_package(Boost ${MujinControllerClient_Boost_VERSION} EXACT COMPONENTS thread date_time filesystem)

set(MUJINVISION_LINK_DIRS)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR COMPILER_IS_CLANG )
  set(CMAKE_CXX_FLAGS_OPTIMIZED "-O3 -DNDEBUG -DBOOST_DISABLE_ASSERTS -D_SECURE_SCL=0") # this practically removes all checks making it a very dangerous options to play with
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -D_DEBUG")

  add_definitions(-fPIC) # this is a very important switch and some libraries seem not to have it....
  set(EXTRA_COMPILE_FLAGS "-fPIC")
else()
  set(EXTRA_COMPILE_FLAGS "")
endif()

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/mujinvision/config.h IMMEDIATE @ONLY)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# libzmq
find_package(libzmq QUIET)
if (NOT libzmq_FOUND)
  if (MSVC)
    message(STATUS "TODO: set up local MSVC libzmq")
  else()
    message(FATAL_ERROR "could not find libzmq")
  endif()
else()
  set(MUJINVISION_LINK_DIRS ${MUJINVISION_LINK_DIRS} ${libzmq_LIBRARY_DIRS})
  set(MUJINVISION_INCLUDE_LOCAL_DIRS ${MUJINVISION_INCLUDE_LOCAL_DIRS} ${libzmq_INCLUDE_DIRS})
endif()

file(GLOB mujin_header_files ${CMAKE_CURRENT_SOURCE_DIR}/include/mujinvision/*.h)
install(FILES ${mujin_header_files} ${CMAKE_CURRENT_BINARY_DIR}/include/mujinvision/config.h DESTINATION include/mujinvision)

macro(build_sample name)
  add_executable(${name} ${name}.cpp)
  set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${Boost_CFLAGS}" LINK_FLAGS "")
  add_dependencies(${name} libmujinvision)
  target_link_libraries (${name} libmujinvision boost_program_options boost_system)
  install(TARGETS ${name} DESTINATION bin)
endmacro(build_sample)

add_subdirectory(src)
if( OPT_SAMPLES )
  add_subdirectory(samples)
endif()
if( OPT_BUILD_TESTS )
  add_subdirectory(test)
endif()

# add make uninstall capability
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/mujinvision-config.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/mujinvision-config.cmake" IMMEDIATE @ONLY)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/mujinvision-config-version.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/mujinvision-config-version.cmake" IMMEDIATE @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/mujinvision-config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/mujinvision-config-version.cmake" DESTINATION "lib${LIB_SUFFIX}/cmake/${MUJINVISION_CMAKE_INSTALL_DIR}")

