/**
 * @file test_vtk_writer.cpp
 * @brief Tests for write_vtk().
 */
#include <gtest/gtest.h>

#include <stdexcept>

#include "test_helpers.h"
#include "vtk_writer.h"

/// @brief Fixture with a small two-element model and result data.
class WriteVtk : public TempDirTest {
   protected:
    std::vector<node> nodes = {{0, 0}, {2, 0}, {1, 1.5}};  ///< Test nodes.
    std::vector<elem> elements = {{1, 2, 1.0, 1.0},
                                  {2, 3, 1.0, 1.0}};  ///< Test elements.
    std::vector<double> displacements = {0,    0,   0.25,
                                         -0.5, 1.5, 2};  ///< Nodal results.
    std::vector<double> axial_stresses = {100, -42.5};   ///< Element results.
};

/// @brief The complete file content matches the legacy VTK layout.
TEST_F(WriteVtk, WritesExpectedLegacyAsciiFile) {
    auto output_path = scratch_dir_ / "out.vtk";
    write_vtk(output_path.string(), nodes, elements, displacements,
              axial_stresses);

    const std::string expected =
        "# vtk DataFile Version 3.0\n"
        "truss solver output\n"
        "ASCII\n"
        "DATASET UNSTRUCTURED_GRID\n"
        "POINTS 3 float\n"
        "0 0 0.0\n"
        "2 0 0.0\n"
        "1 1.5 0.0\n"
        "CELLS 2 6\n"
        "2 0 1\n"
        "2 1 2\n"
        "CELL_TYPES 2\n"
        "3\n"
        "3\n"
        "POINT_DATA 3\n"
        "VECTORS Displacement float\n"
        "0 0 0.0\n"
        "0.25 -0.5 0.0\n"
        "1.5 2 0.0\n"
        "CELL_DATA 2\n"
        "SCALARS Axial_Stress float 1\n"
        "LOOKUP_TABLE default\n"
        "100\n"
        "-42.5\n";
    EXPECT_EQ(read_file(output_path), expected);
}

/// @brief Displacements read back from the file equal those written.
TEST_F(WriteVtk, DisplacementsRoundTrip) {
    auto output_path = scratch_dir_ / "out.vtk";
    write_vtk(output_path.string(), nodes, elements, displacements,
              axial_stresses);
    EXPECT_EQ(read_vtk_displacements(output_path, 3), displacements);
}

/// @brief Missing parent directories are created.
TEST_F(WriteVtk, CreatesMissingParentDirectories) {
    auto output_path = scratch_dir_ / "a" / "b" / "out.vtk";
    write_vtk(output_path.string(), nodes, elements, displacements,
              axial_stresses);
    EXPECT_TRUE(fs::exists(output_path));
}

/// @brief Writing to a path that is a directory throws.
TEST_F(WriteVtk, ThrowsWhenPathIsADirectory) {
    EXPECT_THROW(write_vtk(scratch_dir_.string(), nodes, elements,
                           displacements, axial_stresses),
                 std::runtime_error);
}

/// @brief Writing below a regular file (as if it were a directory) throws.
TEST_F(WriteVtk, ThrowsWhenParentIsAFile) {
    auto blocking_file = scratch_dir_ / "file";
    write_file(blocking_file, "x");
    EXPECT_THROW(write_vtk((blocking_file / "out.vtk").string(), nodes,
                           elements, displacements, axial_stresses),
                 std::runtime_error);
}
