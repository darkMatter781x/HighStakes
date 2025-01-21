# Copied mostly from LemLib/LemLib: https://github.com/LemLib/LemLib/blob/8d2955a0955ded2b7fddf4f4522b7f1d5f180380/CMakeLists.txt
# Which is licensed under the MIT license: https://github.com/LemLib/LemLib/blob/refactor/LICENSE

cmake_minimum_required(VERSION 3.14)
project(HighStakes781x)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BUILD_GMOCK ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow -Wnull-dereference -Wno-psabi -Wno-unused-function -pthread -g -O0 -fprofile-arcs -ftest-coverage -D THREADS_STD -D GTEST")

enable_testing()

# find_package(GTest CONFIG REQUIRED)
include(FetchContent)
FetchContent_Declare(
  googletest
  DOWNLOAD_EXTRACT_TIMESTAMP true
  # Specify the commit you depend on and update it regularly.
  URL https://github.com/google/googletest/archive/b514bdc898e2951020cbdca1304b75f5950d1f59.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

include_directories(include include/packages)
file(GLOB_RECURSE TEST_SOURCES "*.test.cpp")
file(GLOB_RECURSE EIGEN_HEADERS "include/packages/Eigen/*")

add_executable(HighStakes781x
  ${TEST_SOURCES}
  ${EIGEN_HEADERS}
  
  src/pf/util/lines.cpp
  src/pf/util/stats.cpp
  src/pf/util/lines.test.cpp

  include/pf/util/stats.h
  include/pf/util/lines.h
)

# Link against gtest
target_link_libraries(HighStakes781x gtest_main)
target_link_libraries(HighStakes781x gmock_main)

include(GoogleTest)
gtest_discover_tests(HighStakes781x)