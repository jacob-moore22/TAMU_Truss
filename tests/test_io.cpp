/**
 * @file test_io.cpp
 * @brief Tests for read_input().
 */
#include <gtest/gtest.h>

#include <stdexcept>

#include "io.h"
#include "test_helpers.h"

/// @brief Fixture for read_input() tests that need temporary input files.
class ReadInput : public TempDirTest {};

/// @brief Every field of the triangle example is parsed correctly.
TEST_F(ReadInput, ParsesTriangleExample) {
    model truss_model = read_input(example_path("input_triangle.txt"));

    ASSERT_EQ(truss_model.nodes.size(), 3u);
    EXPECT_DOUBLE_EQ(truss_model.nodes[0].x, 0.0);
    EXPECT_DOUBLE_EQ(truss_model.nodes[0].y, 0.0);
    EXPECT_DOUBLE_EQ(truss_model.nodes[1].x, 10.0);
    EXPECT_DOUBLE_EQ(truss_model.nodes[1].y, 0.0);
    EXPECT_DOUBLE_EQ(truss_model.nodes[2].x, 5.0);
    EXPECT_DOUBLE_EQ(truss_model.nodes[2].y, 10.0);

    ASSERT_EQ(truss_model.elements.size(), 3u);
    EXPECT_EQ(truss_model.elements[1].start_node, 2);
    EXPECT_EQ(truss_model.elements[1].end_node, 3);
    EXPECT_DOUBLE_EQ(truss_model.elements[1].area, 1.5);
    EXPECT_DOUBLE_EQ(truss_model.elements[1].youngs_modulus, 200e9);

    ASSERT_EQ(truss_model.boundary_conditions.size(), 3u);
    EXPECT_EQ(truss_model.boundary_conditions[2].node_id, 2);
    EXPECT_EQ(truss_model.boundary_conditions[2].direction, 2);
    EXPECT_DOUBLE_EQ(truss_model.boundary_conditions[2].displacement, 0.0);

    ASSERT_EQ(truss_model.forces.size(), 2u);
    EXPECT_EQ(truss_model.forces[0].node_id, 3);
    EXPECT_EQ(truss_model.forces[0].direction, 1);
    EXPECT_DOUBLE_EQ(truss_model.forces[0].magnitude, 5000.0);
    EXPECT_DOUBLE_EQ(truss_model.forces[1].magnitude, -10000.0);

    EXPECT_EQ(truss_model.load_steps, 10);
}

/// @brief The larger examples load with the expected entry counts.
TEST_F(ReadInput, ParsesFinkAndHoweExamples) {
    model fink_model = read_input(example_path("fink_truss.txt"));
    EXPECT_EQ(fink_model.nodes.size(), 7u);
    EXPECT_EQ(fink_model.elements.size(), 11u);
    EXPECT_EQ(fink_model.boundary_conditions.size(), 4u);
    EXPECT_EQ(fink_model.forces.size(), 1u);
    EXPECT_EQ(fink_model.load_steps, 20);

    model howe_model = read_input(example_path("howe_truss.txt"));
    EXPECT_EQ(howe_model.nodes.size(), 10u);
    EXPECT_EQ(howe_model.elements.size(), 17u);
    EXPECT_EQ(howe_model.boundary_conditions.size(), 4u);
    EXPECT_EQ(howe_model.forces.size(), 5u);
    EXPECT_EQ(howe_model.load_steps, 100);
}

/// @brief Comment lines and blank lines are ignored.
TEST_F(ReadInput, SkipsCommentsAndBlankLines) {
    auto input_path = scratch_dir_ / "in.txt";
    write_file(input_path,
               "# leading comment\n\n*NODES\n# ID X Y\n\n1 1.0 2.0\n   \n"
               "# trailing comment\n");
    model truss_model = read_input(input_path.string());
    ASSERT_EQ(truss_model.nodes.size(), 1u);
    EXPECT_DOUBLE_EQ(truss_model.nodes[0].x, 1.0);
    EXPECT_DOUBLE_EQ(truss_model.nodes[0].y, 2.0);
}

/// @brief Nodes are stored at index ID - 1 regardless of file order.
TEST_F(ReadInput, PlacesOutOfOrderNodesById) {
    auto input_path = scratch_dir_ / "in.txt";
    write_file(input_path, "*NODES\n3 30.0 3.0\n1 10.0 1.0\n2 20.0 2.0\n");
    model truss_model = read_input(input_path.string());
    ASSERT_EQ(truss_model.nodes.size(), 3u);
    EXPECT_DOUBLE_EQ(truss_model.nodes[0].x, 10.0);
    EXPECT_DOUBLE_EQ(truss_model.nodes[1].x, 20.0);
    EXPECT_DOUBLE_EQ(truss_model.nodes[2].x, 30.0);
}

/// @brief Element IDs are discarded; elements keep file order.
TEST_F(ReadInput, IgnoresElementIdsAndKeepsFileOrder) {
    auto input_path = scratch_dir_ / "in.txt";
    write_file(input_path, "*ELEMENTS\n9 1 2 1.0 5.0\n4 2 3 2.0 6.0\n");
    model truss_model = read_input(input_path.string());
    ASSERT_EQ(truss_model.elements.size(), 2u);
    EXPECT_EQ(truss_model.elements[0].start_node, 1);
    EXPECT_DOUBLE_EQ(truss_model.elements[0].youngs_modulus, 5.0);
    EXPECT_EQ(truss_model.elements[1].start_node, 2);
    EXPECT_DOUBLE_EQ(truss_model.elements[1].area, 2.0);
}

/// @brief Lowercase section headers are not recognised.
TEST_F(ReadInput, SectionHeadersAreCaseSensitive) {
    auto input_path = scratch_dir_ / "in.txt";
    write_file(input_path, "*nodes\n1 0.0 0.0\n");
    model truss_model = read_input(input_path.string());
    EXPECT_TRUE(truss_model.nodes.empty());
}

/// @brief A file without *LOAD_STEPS uses a single load step.
TEST_F(ReadInput, LoadStepsDefaultsToOne) {
    auto input_path = scratch_dir_ / "in.txt";
    write_file(input_path, "*NODES\n1 0.0 0.0\n");
    EXPECT_EQ(read_input(input_path.string()).load_steps, 1);
}

/// @brief A nonexistent input file throws.
TEST_F(ReadInput, ThrowsOnMissingFile) {
    EXPECT_THROW(read_input((scratch_dir_ / "does_not_exist.txt").string()),
                 std::runtime_error);
}
