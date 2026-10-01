#include <gtest/gtest.h>

#include <stdexcept>

#include "test_helpers.h"
#include "vtk_writer.h"

class WriteVtk : public TempDirTest {
   protected:
    std::vector<node> nodes = {{0, 0}, {2, 0}, {1, 1.5}};
    std::vector<elem> elems = {{1, 2, 1.0, 1.0}, {2, 3, 1.0, 1.0}};
    std::vector<double> u = {0, 0, 0.25, -0.5, 1.5, 2};
    std::vector<double> stresses = {100, -42.5};
};

TEST_F(WriteVtk, WritesExpectedLegacyAsciiFile) {
    auto path = dir_ / "out.vtk";
    write_vtk(path.string(), nodes, elems, u, stresses);

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
    EXPECT_EQ(read_file(path), expected);
}

TEST_F(WriteVtk, DisplacementsRoundTrip) {
    auto path = dir_ / "out.vtk";
    write_vtk(path.string(), nodes, elems, u, stresses);
    EXPECT_EQ(read_vtk_displacements(path, 3), u);
}

TEST_F(WriteVtk, CreatesMissingParentDirectories) {
    auto path = dir_ / "a" / "b" / "out.vtk";
    write_vtk(path.string(), nodes, elems, u, stresses);
    EXPECT_TRUE(fs::exists(path));
}

TEST_F(WriteVtk, ThrowsWhenPathIsADirectory) {
    EXPECT_THROW(write_vtk(dir_.string(), nodes, elems, u, stresses),
                 std::runtime_error);
}

TEST_F(WriteVtk, ThrowsWhenParentIsAFile) {
    auto blocker = dir_ / "file";
    write_file(blocker, "x");
    EXPECT_THROW(
        write_vtk((blocker / "out.vtk").string(), nodes, elems, u, stresses),
        std::runtime_error);
}
