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
include_directories(${MUJINVISION_INCLUDE_LOCAL_DIRS} ${libzmq_INCLUDE_DIRS} ${MujinControllerClient_INCLUDE_DIRS})
link_directories(${MUJINVISION_LINK_DIRS} ${libzmq_LIBRARY_DIRS} ${MujinControllerClient_LIBRARY_DIRS})

set(SOURCE_FILES mujinzmq.cpp visionmanager.cpp)

add_library(libmujinvision SHARED ${SOURCE_FILES})

set_target_properties(libmujinvision PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS} ${Boost_CFLAGS} -DMUJINVISION_DLL_EXPORTS -DMUJINVISION_DLL ${MujinControllerClient_CXX_FLAGS}" LINK_FLAGS "${MujinControllerClient_LINK_FLAGS}"
  OUTPUT_NAME mujinvision${MUJINVISION_LIBRARY_SUFFIX}
  SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
  VERSION ${MUJINVISION_VERSION}
  CLEAN_DIRECT_OUTPUT 1
  )
target_link_libraries(libmujinvision ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY}  ${EXTRA_LIBRARIES} ${libzmq_LIBRARIES} ${MujinControllerClient_LIBRARIES})
if( MSVC )
  message(STATUS "TODO: install for MSVC")
else()
  install(TARGETS libmujinvision DESTINATION lib${LIB_SUFFIX})
endif()

if( OPT_BUILD_STATIC )
  # visual studio needs static lib built
  if( MSVC )
    message(STATUS "TODO: install for MSVC")
  else()
    set(LIBMUJINVISION_NAME mujinvision${MUJINVISION_LIBRARY_SUFFIX})
  endif()

  add_library(libmujinvision_static STATIC ${SOURCE_FILES})
  set_target_properties(libmujinvision_static PROPERTIES OUTPUT_NAME ${LIBMUJINVISION_NAME}
                                           SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
                                           VERSION ${MUJINVISION_VERSION}
                                           CLEAN_DIRECT_OUTPUT 1
                                           COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS} ${Boost_CFLAGS} -DMUJINVISION_DLL_EXPORTS -DMUJINVISION_DLL ${MujinControllerClient_CXX_FLAGS}"
                                           LINK_FLAGS "${MujinControllerClient_LINK_FLAGS}")
  
  target_link_libraries(libmujinvision_static ${EXTRA_LIBRARIES} ${libzmq_LIBRARIES} ${MujinControllerClient_LIBRARIES})
  install(TARGETS libmujinvision_static DESTINATION lib${LIB_SUFFIX})
endif()


install(TARGETS libmujinvision DESTINATION lib)
set(MUJIN_VISION_INCLUDE_LOCAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
file(GLOB LIB_HEADER_FILES ${MUJIN_VISION_INCLUDE_LOCAL_DIR}/mujinvision/*.h)
install(FILES ${LIB_HEADER_FILES} DESTINATION include/mujinvision)
