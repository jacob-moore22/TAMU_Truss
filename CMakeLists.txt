cmake_minimum_required(VERSION 3.20)
project(TAMU_Truss CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_library(truss_core STATIC
    src/io.cpp
    src/solver.cpp
    src/vtk_writer.cpp
    src/driver.cpp
)
target_include_directories(truss_core PUBLIC include)

add_executable(truss_solver src/main.cpp)
target_link_libraries(truss_solver PRIVATE truss_core)
# Place the binary at the repo root so `./truss_solver examples/...` works.
set_target_properties(truss_solver PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
)

option(TRUSS_BUILD_TESTS "Build unit tests" ON)
if(TRUSS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "Generating Doxygen HTML in build/doxygen/html"
        VERBATIM
    )
endif()
