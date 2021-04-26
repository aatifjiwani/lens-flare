# Original file Copyright CMU462 Fall 2015: 
# Kayvon Fatahalian, Keenan Crane,
# Sky Gao, Bryce Summers, Michael Choquette.
cmake_minimum_required(VERSION 3.1)

project(CGL LANGUAGES C CXX VERSION 2020.0)

#-------------------------------------------------------------------------------
# Build options
#-------------------------------------------------------------------------------
option(CGL_BUILD_DEBUG    "Build for debug"          OFF)
option(CGL_BUILD_SHARED   "Build shared libraries"   OFF)
option(CGL_BUILD_DOCS     "Build documentation"      OFF)
option(CGL_BUILD_TESTS    "Build tests programs"     OFF)
option(CGL_BUILD_EXAMPLES "Build examples"           OFF)

if(BUILD_DEBUG)
    set(CGL_BUILD_DEBUG ON)
endif()

if(BUILD_DOCS)
    set(CGL_BUILD_DOCS ON)
endif()

#-------------------------------------------------------------------------------
# CMake options
#-------------------------------------------------------------------------------
if(CGL_BUILD_DEBUG)
    set(CMAKE_BUILD_TYPE Debug)
endif()

#-------------------------------------------------------------------------------
# Compiler-specific Options
#-------------------------------------------------------------------------------

include(find_avx.cmake)
CHECK_FOR_AVX()

if(${CMAKE_CXX_COMPILER_ID} STREQUAL Clang OR ${CMAKE_CXX_COMPILER_ID} STREQUAL AppleClang)
    set(CGL_CXX_FLAGS "-std=c++11 -m64 -fPIC")
    set(CGL_CXX_FLAGS "${CGL_CXX_FLAGS} -O3")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

if(${CMAKE_CXX_COMPILER_ID} STREQUAL GNU)
    set(CGL_CXX_FLAGS "-std=c++11 -m64 -fPIC")
    set(CGL_CXX_FLAGS "${CGL_CXX_FLAGS} -O3")
    if (CGL_BUILD_DEBUG)
        set(CGL_CXX_FLAGS "${CGL_CXX_FLAGS} -ggdb")
    endif()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

if(${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
    set(CGL_CXX_FLAGS "/std:c++17 /wd4838 /wd4305 /wd4244 /wd4267")
    set(CGL_CXX_FLAGS "${CGL_CXX_FLAGS} /O2")
endif()

set(CGL_CXX_FLAGS "${CGL_CXX_FLAGS} ${AVX_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CGL_CXX_FLAGS}")

#-------------------------------------------------------------------------------
# Create target: CGL
#-------------------------------------------------------------------------------
set(CGL_SOURCE
    src/vector2D.cpp
    src/vector3D.cpp
    src/vector4D.cpp
    src/matrix3x3.cpp
    src/matrix4x4.cpp
    src/quaternion.cpp
    src/complex.cpp
    src/color.cpp
    src/osdtext.cpp
    src/osdfont.cpp
    src/viewer.cpp
    src/base64.cpp
    src/lodepng.cpp
    src/tinyxml2.cpp
    src/path.cpp
)

add_library(CGL STATIC ${CGL_SOURCE})

set(CGL_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/CGL
)
target_include_directories(CGL PUBLIC ${CGL_INCLUDE_DIRS})

#-------------------------------------------------------------------------------
# Find dependencies
#-------------------------------------------------------------------------------

# Threads
find_package(Threads REQUIRED)

# OpenGL
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)

# Freetype
find_package(Freetype REQUIRED)
target_include_directories(CGL PRIVATE ${FREETYPE_INCLUDE_DIRS})
target_link_libraries(CGL PRIVATE ${FREETYPE_LIBRARIES})

# GLEW
add_definitions(-DGLEW_STATIC)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/deps/glew")
target_include_directories(CGL PUBLIC ${GLEW_INCLUDE_DIRS})
target_link_libraries(CGL PUBLIC ${GLEW_LIBRARIES})

# GLFW
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/deps/glfw")
target_link_libraries(CGL PUBLIC glfw)
target_link_libraries(CGL PUBLIC OpenGL::GL)

#-------------------------------------------------------------------------------
# Add subdirectories
#-------------------------------------------------------------------------------

# CGL tests source directory
if(CGL_BUILD_TESTS)
  add_subdirectory(tests)
endif()

# CGL exmaples source directory
if(CGL_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# CGL documentation directory
if(CGL_BUILD_DOCS)
  find_package(DOXYGEN)
  if(DOXYGEN_FOUND AND CGL_BUILD_DOCS)
    add_subdirectory(docs)
  endif()
endif()

#-------------------------------------------------------------------------------
# Packing
#-------------------------------------------------------------------------------

# Install settings
set(CMAKE_INSTALL_PREFIX "${CGL_SOURCE_DIR}")

set(CGL_CXX_FLAGS "${CGL_CXX_FLAGS}" PARENT_SCOPE)
