cmake_minimum_required(VERSION 3.14)
project(TAMU_Truss CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_library(truss_core STATIC
    src/solver.cpp
    src/io.cpp
    src/vtk_writer.cpp
)
target_include_directories(truss_core PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_executable(truss_solver src/main.cpp)
target_link_libraries(truss_solver PRIVATE truss_core)

# --- Convenience developer targets (mirror the old Makefile) ---

find_program(CLANG_FORMAT_EXE clang-format)
if(CLANG_FORMAT_EXE)
    file(GLOB_RECURSE FORMAT_SOURCES
        ${CMAKE_SOURCE_DIR}/src/*.cpp
        ${CMAKE_SOURCE_DIR}/include/*.h
        ${CMAKE_SOURCE_DIR}/tests/*.cpp
    )
    add_custom_target(format
        COMMAND ${CLANG_FORMAT_EXE} -i ${FORMAT_SOURCES}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Running clang-format over src/, include/, tests/"
    )
endif()

find_program(DOXYGEN_EXE doxygen)
if(DOXYGEN_EXE)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXE} ${CMAKE_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating Doxygen HTML docs into docs/html"
    )
endif()

# --- Tests ---

option(BUILD_TESTING "Build unit tests" ON)
if(BUILD_TESTING)
    include(FetchContent)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
    )
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    include(GoogleTest)
    add_subdirectory(tests)
endif()
