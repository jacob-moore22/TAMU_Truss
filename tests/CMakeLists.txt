include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
# For Windows: prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(truss_tests
    test_types.cpp
    test_solver.cpp
    test_io.cpp
    test_vtk_writer.cpp
    test_driver.cpp
)
target_link_libraries(truss_tests PRIVATE truss_core GTest::gtest_main)
target_compile_definitions(truss_tests PRIVATE
    TRUSS_EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/examples"
)

include(GoogleTest)
gtest_discover_tests(truss_tests)
