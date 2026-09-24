cmake_minimum_required(VERSION 3.14)
project(TAMU_Truss CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(TRUSS_BUILD_TESTS "Build unit tests (downloads GoogleTest)" OFF)

add_library(truss_core
    src/io.cpp
    src/solver.cpp
    src/vtk_writer.cpp
    src/driver.cpp
)
target_include_directories(truss_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(truss_solver src/main.cpp)
target_link_libraries(truss_solver PRIVATE truss_core)

if(TRUSS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
