add_executable(truss_tests
    test_solver.cpp
    test_vtk_writer.cpp
    test_integration.cpp
)
target_link_libraries(truss_tests PRIVATE truss_core GTest::gtest_main)

gtest_discover_tests(truss_tests)
