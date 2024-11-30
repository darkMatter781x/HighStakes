# Copied mostly from LemLib/LemLib: https://github.com/LemLib/LemLib/blob/8d2955a0955ded2b7fddf4f4522b7f1d5f180380/CMakeLists.txt
# Which is licensed under the MIT license: https://github.com/LemLib/LemLib/blob/refactor/LICENSE

cmake_minimum_required(VERSION 3.6)
project(HighStakes781x)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow -Wnull-dereference -Wno-psabi -Wno-unused-function -pthread -g -O0 -fprofile-arcs -ftest-coverage -D THREADS_STD -D GTEST")

enable_testing()

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in external-dependencies/CMakeLists.txt)
message("${CMAKE_GENERATOR}")
message("cmake command" ${CMAKE_COMMAND})

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external-dependencies )
if(result)
    message(FATAL_ERROR "CMake step for dependencies failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external-dependencies )
if(result)
    message(FATAL_ERROR "Build step for dependencies failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

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

  include/pf/util/stats.h
  include/pf/util/lines.h
)

# Link against gtest
target_link_libraries(HighStakes781x gtest_main)

include(GoogleTest)
gtest_discover_tests(HighStakes781x)