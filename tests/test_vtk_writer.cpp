#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "vtk_writer.h"

namespace {

class VtkWriterTest : public ::testing::Test {
protected:
    void SetUp() override {
        scratch_dir_ =
            std::filesystem::path(::testing::TempDir()) /
            ("truss_vtk_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
             "_" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
        std::filesystem::remove_all(scratch_dir_);
        std::filesystem::create_directories(scratch_dir_);
    }

    void TearDown() override { std::filesystem::remove_all(scratch_dir_); }

    std::filesystem::path scratch_dir_;
};

std::vector<std::string> ReadLines(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) lines.push_back(line);
    return lines;
}

std::vector<double> ParseDoubles(const std::string& line) {
    std::istringstream iss(line);
    std::vector<double> values;
    double v;
    while (iss >> v) values.push_back(v);
    return values;
}

size_t FindLineIndex(const std::vector<std::string>& lines, const std::string& exact) {
    for (size_t i = 0; i < lines.size(); ++i)
        if (lines[i] == exact) return i;
    return lines.size();
}

}  // namespace

TEST_F(VtkWriterTest, WritesExpectedStructureAndContent) {
    std::vector<node> nodes = {{0.0, 0.0}, {1.0, 2.0}};
    std::vector<elem> elements = {{1, 2, 1.0, 1.0}};
    std::vector<double> displacements = {0.1, 0.2, 0.3, 0.4};
    std::vector<double> stresses = {123.5};

    auto out_path = scratch_dir_ / "out.vtk";
    write_vtk(out_path.string(), nodes, elements, displacements, stresses);

    ASSERT_TRUE(std::filesystem::exists(out_path));
    auto lines = ReadLines(out_path);
    ASSERT_EQ(lines.size(), 19u);  // 18 header/structure lines + 1 stress value

    EXPECT_EQ(lines[0], "# vtk DataFile Version 3.0");
    EXPECT_EQ(lines[1], "truss solver output");
    EXPECT_EQ(lines[2], "ASCII");
    EXPECT_EQ(lines[3], "DATASET UNSTRUCTURED_GRID");

    EXPECT_EQ(lines[4], "POINTS 2 float");
    auto p0 = ParseDoubles(lines[5]);
    ASSERT_EQ(p0.size(), 3u);
    EXPECT_DOUBLE_EQ(p0[0], 0.0);
    EXPECT_DOUBLE_EQ(p0[1], 0.0);
    EXPECT_DOUBLE_EQ(p0[2], 0.0);
    auto p1 = ParseDoubles(lines[6]);
    ASSERT_EQ(p1.size(), 3u);
    EXPECT_DOUBLE_EQ(p1[0], 1.0);
    EXPECT_DOUBLE_EQ(p1[1], 2.0);
    EXPECT_DOUBLE_EQ(p1[2], 0.0);

    EXPECT_EQ(lines[7], "CELLS 1 3");
    EXPECT_EQ(lines[8], "2 0 1");  // node1_id-1=0, node2_id-1=1

    EXPECT_EQ(lines[9], "CELL_TYPES 1");
    EXPECT_EQ(lines[10], "3");

    EXPECT_EQ(lines[11], "POINT_DATA 2");
    EXPECT_EQ(lines[12], "VECTORS Displacement float");
    auto d0 = ParseDoubles(lines[13]);
    ASSERT_EQ(d0.size(), 3u);
    EXPECT_NEAR(d0[0], 0.1, 1e-9);
    EXPECT_NEAR(d0[1], 0.2, 1e-9);
    EXPECT_NEAR(d0[2], 0.0, 1e-9);
    auto d1 = ParseDoubles(lines[14]);
    ASSERT_EQ(d1.size(), 3u);
    EXPECT_NEAR(d1[0], 0.3, 1e-9);
    EXPECT_NEAR(d1[1], 0.4, 1e-9);
    EXPECT_NEAR(d1[2], 0.0, 1e-9);

    EXPECT_EQ(lines[15], "CELL_DATA 1");
    EXPECT_EQ(lines[16], "SCALARS Axial_Stress float 1");
    EXPECT_EQ(lines[17], "LOOKUP_TABLE default");
}

TEST_F(VtkWriterTest, WritesStressValueAfterLookupTable) {
    std::vector<node> nodes = {{0.0, 0.0}, {1.0, 0.0}};
    std::vector<elem> elements = {{1, 2, 1.0, 1.0}};
    std::vector<double> displacements = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> stresses = {123.5};

    auto out_path = scratch_dir_ / "out.vtk";
    write_vtk(out_path.string(), nodes, elements, displacements, stresses);

    auto lines = ReadLines(out_path);
    ASSERT_EQ(lines.size(), 19u);
    auto s = ParseDoubles(lines[18]);
    ASSERT_EQ(s.size(), 1u);
    EXPECT_NEAR(s[0], 123.5, 1e-9);
}

TEST_F(VtkWriterTest, CreatesMissingParentDirectories) {
    auto out_path = scratch_dir_ / "nested" / "does" / "not" / "exist" / "out.vtk";
    ASSERT_FALSE(std::filesystem::exists(out_path.parent_path()));

    std::vector<node> nodes = {{0.0, 0.0}, {1.0, 0.0}};
    std::vector<elem> elements = {{1, 2, 1.0, 1.0}};
    std::vector<double> displacements = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> stresses = {0.0};

    write_vtk(out_path.string(), nodes, elements, displacements, stresses);

    EXPECT_TRUE(std::filesystem::exists(out_path));
}

TEST_F(VtkWriterTest, ThrowsWhenOutputPathCannotBeOpened) {
    // A path that is itself an existing directory can never be opened as an
    // ofstream, so write_vtk should hit its own explicit runtime_error.
    auto blocked_dir = scratch_dir_ / "blocked_dir";
    std::filesystem::create_directory(blocked_dir);

    std::vector<node> nodes = {{0.0, 0.0}, {1.0, 0.0}};
    std::vector<elem> elements = {{1, 2, 1.0, 1.0}};
    std::vector<double> displacements = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> stresses = {0.0};

    EXPECT_THROW(write_vtk(blocked_dir.string(), nodes, elements, displacements, stresses),
                 std::runtime_error);
}

TEST_F(VtkWriterTest, MultipleElementsAndNodesPreserveOrder) {
    std::vector<node> nodes = {{0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}};
    std::vector<elem> elements = {
        {1, 2, 1.0, 1.0},
        {2, 3, 1.0, 1.0},
        {1, 3, 1.0, 1.0},
    };
    std::vector<double> displacements = {0.0, 0.0, 0.1, 0.0, 0.1, 0.1};
    std::vector<double> stresses = {10.0, -20.0, 30.0};

    auto out_path = scratch_dir_ / "out.vtk";
    write_vtk(out_path.string(), nodes, elements, displacements, stresses);

    auto lines = ReadLines(out_path);
    size_t cells_line = FindLineIndex(lines, "CELLS 3 9");
    ASSERT_LT(cells_line + 3, lines.size()) << "CELLS 3 9 header not found";
    EXPECT_EQ(lines[cells_line + 1], "2 0 1");
    EXPECT_EQ(lines[cells_line + 2], "2 1 2");
    EXPECT_EQ(lines[cells_line + 3], "2 0 2");
    EXPECT_EQ(lines[cells_line + 4], "CELL_TYPES 3");
}
