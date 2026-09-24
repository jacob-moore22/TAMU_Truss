cmake_minimum_required(VERSION 3.14)
project(TAMU_Truss CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(TRUSS_BUILD_TESTS "Build unit tests (downloads GoogleTest)" OFF)
option(TRUSS_BUILD_DOCS "Build Doxygen HTML documentation" OFF)

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

if(TRUSS_BUILD_DOCS)
    find_package(Doxygen REQUIRED OPTIONAL_COMPONENTS dot)

    set(DOXYGEN_PROJECT_NAME "TAMU Truss")
    set(DOXYGEN_PROJECT_BRIEF "2D truss solver using the direct stiffness method")
    set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs)
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE ${CMAKE_CURRENT_SOURCE_DIR}/README.md)
    set(DOXYGEN_STRIP_FROM_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_GENERATE_LATEX NO)
    # Render the @f$...@f$ equations in the browser; no LaTeX install needed.
    set(DOXYGEN_USE_MATHJAX YES)
    # Document helpers in anonymous namespaces inside the .cpp files too.
    set(DOXYGEN_EXTRACT_ANON_NSPACES YES)
    set(DOXYGEN_SOURCE_BROWSER YES)
    set(DOXYGEN_WARN_IF_UNDOCUMENTED YES)
    set(DOXYGEN_WARN_NO_PARAMDOC YES)
    # CI turns this on so undocumented code fails the build.
    option(TRUSS_DOCS_WARN_AS_ERROR "Treat Doxygen warnings as errors" OFF)
    if(TRUSS_DOCS_WARN_AS_ERROR)
        set(DOXYGEN_WARN_AS_ERROR FAIL_ON_WARNINGS)
    endif()
    if(DOXYGEN_DOT_FOUND)
        set(DOXYGEN_HAVE_DOT YES)
        set(DOXYGEN_CALL_GRAPH YES)
        set(DOXYGEN_CALLER_GRAPH YES)
    endif()

    doxygen_add_docs(docs
        ${CMAKE_CURRENT_SOURCE_DIR}/README.md
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ALL
        COMMENT "Generating Doxygen HTML documentation"
    )
endif()
