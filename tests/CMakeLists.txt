include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(truss_tests
    test_io.cpp
    test_solver.cpp
    test_vtk_writer.cpp
    test_driver.cpp
)
target_link_libraries(truss_tests PRIVATE truss_core GTest::gtest_main)
target_compile_definitions(truss_tests PRIVATE
    TRUSS_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

include(GoogleTest)
gtest_discover_tests(truss_tests)
