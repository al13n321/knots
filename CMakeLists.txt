cmake_minimum_required(VERSION 3.0.0)
project(knots)

SET (LIBS)

include_directories(
  ./
  ./lib/gl3w/include/
)

IF (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  SET (MSVC 1)
ENDIF ()

include_directories(
  ./
  ./lib/gl3w/include/
)

IF (MSVC)
  SET (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W3 /WX /O2 /MT /D _CRT_SECURE_NO_WARNINGS /EHsc /std:c++17")
  
  add_definitions(-DUSE_OVR)

  include_directories(
    ./lib/glfw-3.3.2.bin.WIN64/include/
    ./lib/ovr_sdk_win_1.43.0/LibOVR/Include/
    ./lib/ovr_sdk_win_1.43.0/LibOVRKernel/Src/
  )
  link_directories(
    ./lib/glfw-3.3.2.bin.WIN64/lib-vc2019/
    ./lib/ovr_sdk_win_1.43.0/LibOVR/Lib/Windows/x64/Release/VS2017/
  )

  SET (LIBS glfw3 OpenGL32 LibOVR)
elseif (APPLE)
  SET (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++17 -g -Wall -Werror -O3")

  include_directories(
    /opt/local/include/
  )

  link_directories(
    /opt/local/lib/
  )

  FIND_LIBRARY(CORE_FOUNDATION_LIBRARY CoreFoundation)
  SET (LIBS glfw ${CORE_FOUNDATION_LIBRARY})
ELSE ()
  message(FATAL_ERROR "Only windows and mac os supported so far. Please update CMakeLists.txt to support your platform.")
ENDIF ()

add_executable(knots
  main.cpp
  gl-util/gl-common.cpp
  gl-util/glfw-util.cpp
  gl-util/shader.cpp
  gl-util/vertex-array.cpp
  gl-util/texture2d.cpp
  util/debug.cpp
  util/exceptions.cpp
  util/mat.cpp
  util/stopwatch.cpp
  lib/gl3w/src/gl3w.c
  sim/render.cpp
  sim/phys.cpp
 "vr/ovr-util.h")

target_link_libraries(knots
  ${LIBS}
)
