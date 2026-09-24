#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "test_helpers.h"
#include "vtk_writer.h"

namespace {

// Formats a double the same way operator<< with default precision does, so
// expected lines match what write_vtk emits.
std::string fmt(double v) {
    std::ostringstream ss;
    ss << v;
    return ss.str();
}

}  // namespace

TEST(WriteVtk, FileStructureForTriangle) {
    temp_dir d;
    model m = triangle_model();
    std::vector<double> u = {0, 0, 1e-7, 0, 5e-7, -2e-7};
    std::vector<double> s = {100.5, -200.25, 0};
    std::string path = d.str() + "/out.vtk";

    write_vtk(path, m.nodes, m.elements, u, s);

    std::vector<std::string> want = {
        "# vtk DataFile Version 3.0",
        "truss solver output",
        "ASCII",
        "DATASET UNSTRUCTURED_GRID",
        "POINTS 3 float",
        "0 0 0.0",
        "10 0 0.0",
        "5 10 0.0",
        "CELLS 3 9",
        "2 0 1",
        "2 1 2",
        "2 0 2",
        "CELL_TYPES 3",
        "3",
        "3",
        "3",
        "POINT_DATA 3",
        "VECTORS Displacement float",
        "0 0 0.0",
        fmt(1e-7) + " 0 0.0",
        fmt(5e-7) + " " + fmt(-2e-7) + " 0.0",
        "CELL_DATA 3",
        "SCALARS Axial_Stress float 1",
        "LOOKUP_TABLE default",
        "100.5",
        "-200.25",
        "0",
    };

    std::vector<std::string> got = read_lines(path);
    ASSERT_EQ(got.size(), want.size());
    for (size_t i = 0; i < want.size(); ++i)
        EXPECT_EQ(got[i], want[i]) << "line " << i + 1;
}

TEST(WriteVtk, CreatesMissingParentDirectories) {
    temp_dir d;
    std::string path = d.str() + "/a/b/c/out.vtk";
    model m = triangle_model();
    std::vector<double> u(6, 0.0), s(3, 0.0);

    EXPECT_NO_THROW(write_vtk(path, m.nodes, m.elements, u, s));
    EXPECT_TRUE(fs::exists(path));
}

TEST(WriteVtk, UnwritablePathThrows) {
    temp_dir d;
    // Make a regular file where the output's parent directory should be.
    std::string blocker = d.write_file("blocker", "x");
    std::string path = blocker + "/out.vtk";
    model m = triangle_model();
    std::vector<double> u(6, 0.0), s(3, 0.0);

    EXPECT_THROW(write_vtk(path, m.nodes, m.elements, u, s), std::exception);
}

TEST(WriteVtk, EmptyGridIsWellFormed) {
    temp_dir d;
    std::string path = d.str() + "/empty.vtk";
    write_vtk(path, {}, {}, {}, {});

    std::vector<std::string> want = {
        "# vtk DataFile Version 3.0",
        "truss solver output",
        "ASCII",
        "DATASET UNSTRUCTURED_GRID",
        "POINTS 0 float",
        "CELLS 0 0",
        "CELL_TYPES 0",
        "POINT_DATA 0",
        "VECTORS Displacement float",
        "CELL_DATA 0",
        "SCALARS Axial_Stress float 1",
        "LOOKUP_TABLE default",
    };
    EXPECT_EQ(read_lines(path), want);
}

TEST(WriteVtk, OverwritesExistingFile) {
    temp_dir d;
    std::string path = d.write_file("out.vtk", "stale content that is long\n");
    write_vtk(path, {}, {}, {}, {});
    std::vector<std::string> lines = read_lines(path);
    ASSERT_FALSE(lines.empty());
    EXPECT_EQ(lines[0], "# vtk DataFile Version 3.0");
    EXPECT_EQ(lines.size(), 12u);
}
