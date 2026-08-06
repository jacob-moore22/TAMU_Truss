add_executable(truss_tests
    test_solver.cpp
    test_io.cpp
    test_vtk_writer.cpp
)
target_link_libraries(truss_tests PRIVATE truss_core GTest::gtest_main)
target_compile_definitions(truss_tests PRIVATE
    TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures"
)

include(GoogleTest)
gtest_discover_tests(truss_tests)
