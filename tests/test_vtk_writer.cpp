#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "vtk_writer.h"

TEST(WriteVtk, WritesExpectedHeaderStructureAndCounts) {
    std::vector<node> nodes = {{0.0, 0.0}, {10.0, 0.0}};
    std::vector<elem> elems = {{1, 2, 1.0, 100.0}};
    std::vector<double> u = {0.0, 0.0, 1.0, 0.0};
    std::vector<double> stresses = {10.0};

    auto path = std::filesystem::temp_directory_path() / "truss_test_structure" / "out.vtk";
    write_vtk(path.string(), nodes, elems, u, stresses);

    std::ifstream f(path);
    ASSERT_TRUE(f.is_open());
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(f, line)) lines.push_back(line);

    ASSERT_GE(lines.size(), 5u);
    EXPECT_EQ(lines[0], "# vtk DataFile Version 3.0");
    EXPECT_EQ(lines[2], "ASCII");
    EXPECT_EQ(lines[3], "DATASET UNSTRUCTURED_GRID");
    EXPECT_EQ(lines[4], "POINTS 2 float");

    bool foundCells = false, foundCellTypes = false, foundPointData = false, foundCellData = false;
    for (const auto &l : lines) {
        if (l.rfind("CELLS", 0) == 0) {
            EXPECT_EQ(l, "CELLS 1 3");
            foundCells = true;
        }
        if (l.rfind("CELL_TYPES", 0) == 0) {
            EXPECT_EQ(l, "CELL_TYPES 1");
            foundCellTypes = true;
        }
        if (l.rfind("POINT_DATA", 0) == 0) {
            EXPECT_EQ(l, "POINT_DATA 2");
            foundPointData = true;
        }
        if (l.rfind("CELL_DATA", 0) == 0) {
            EXPECT_EQ(l, "CELL_DATA 1");
            foundCellData = true;
        }
    }
    EXPECT_TRUE(foundCells);
    EXPECT_TRUE(foundCellTypes);
    EXPECT_TRUE(foundPointData);
    EXPECT_TRUE(foundCellData);

    std::filesystem::remove_all(path.parent_path());
}

TEST(WriteVtk, CreatesNonexistentParentDirectories) {
    auto dir = std::filesystem::temp_directory_path() / "truss_test_nested" / "a" / "b" / "c";
    auto path = dir / "out.vtk";
    std::filesystem::remove_all(std::filesystem::temp_directory_path() / "truss_test_nested");
    ASSERT_FALSE(std::filesystem::exists(dir));

    std::vector<node> nodes = {{0.0, 0.0}};
    std::vector<elem> elems = {};
    std::vector<double> u = {0.0, 0.0};
    std::vector<double> stresses = {};
    write_vtk(path.string(), nodes, elems, u, stresses);

    EXPECT_TRUE(std::filesystem::exists(path));

    std::filesystem::remove_all(std::filesystem::temp_directory_path() / "truss_test_nested");
}

TEST(WriteVtk, DisplacementAndStressValuesAppearInOutput) {
    std::vector<node> nodes = {{0.0, 0.0}, {10.0, 0.0}};
    std::vector<elem> elems = {{1, 2, 1.0, 100.0}};
    std::vector<double> u = {0.0, 0.0, 1.5, 0.0};
    std::vector<double> stresses = {42.5};

    auto path = std::filesystem::temp_directory_path() / "truss_test_values" / "out.vtk";
    write_vtk(path.string(), nodes, elems, u, stresses);

    std::ifstream f(path);
    std::ostringstream buf;
    buf << f.rdbuf();
    std::string content = buf.str();

    EXPECT_NE(content.find("1.5"), std::string::npos);
    EXPECT_NE(content.find("42.5"), std::string::npos);

    std::filesystem::remove_all(path.parent_path());
}
