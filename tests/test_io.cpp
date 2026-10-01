#include <gtest/gtest.h>

#include <stdexcept>

#include "io.h"
#include "test_helpers.h"

class ReadInput : public TempDirTest {};

TEST_F(ReadInput, ParsesTriangleExample) {
    model m = read_input(example_path("input_triangle.txt"));

    ASSERT_EQ(m.nodes.size(), 3u);
    EXPECT_DOUBLE_EQ(m.nodes[0].x, 0.0);
    EXPECT_DOUBLE_EQ(m.nodes[0].y, 0.0);
    EXPECT_DOUBLE_EQ(m.nodes[1].x, 10.0);
    EXPECT_DOUBLE_EQ(m.nodes[1].y, 0.0);
    EXPECT_DOUBLE_EQ(m.nodes[2].x, 5.0);
    EXPECT_DOUBLE_EQ(m.nodes[2].y, 10.0);

    ASSERT_EQ(m.elems.size(), 3u);
    EXPECT_EQ(m.elems[1].n1, 2);
    EXPECT_EQ(m.elems[1].n2, 3);
    EXPECT_DOUBLE_EQ(m.elems[1].a, 1.5);
    EXPECT_DOUBLE_EQ(m.elems[1].e, 200e9);

    ASSERT_EQ(m.bcs.size(), 3u);
    EXPECT_EQ(m.bcs[2].node, 2);
    EXPECT_EQ(m.bcs[2].dof, 2);
    EXPECT_DOUBLE_EQ(m.bcs[2].val, 0.0);

    ASSERT_EQ(m.forces.size(), 2u);
    EXPECT_EQ(m.forces[0].node, 3);
    EXPECT_EQ(m.forces[0].dof, 1);
    EXPECT_DOUBLE_EQ(m.forces[0].val, 5000.0);
    EXPECT_DOUBLE_EQ(m.forces[1].val, -10000.0);

    EXPECT_EQ(m.load_steps, 10);
}

TEST_F(ReadInput, ParsesFinkAndHoweExamples) {
    model fink = read_input(example_path("fink_truss.txt"));
    EXPECT_EQ(fink.nodes.size(), 7u);
    EXPECT_EQ(fink.elems.size(), 11u);
    EXPECT_EQ(fink.bcs.size(), 4u);
    EXPECT_EQ(fink.forces.size(), 1u);
    EXPECT_EQ(fink.load_steps, 20);

    model howe = read_input(example_path("howe_truss.txt"));
    EXPECT_EQ(howe.nodes.size(), 10u);
    EXPECT_EQ(howe.elems.size(), 17u);
    EXPECT_EQ(howe.bcs.size(), 4u);
    EXPECT_EQ(howe.forces.size(), 5u);
    EXPECT_EQ(howe.load_steps, 100);
}

TEST_F(ReadInput, SkipsCommentsAndBlankLines) {
    auto path = dir_ / "in.txt";
    write_file(path,
               "# leading comment\n\n*NODES\n# ID X Y\n\n1 1.0 2.0\n   \n"
               "# trailing comment\n");
    model m = read_input(path.string());
    ASSERT_EQ(m.nodes.size(), 1u);
    EXPECT_DOUBLE_EQ(m.nodes[0].x, 1.0);
    EXPECT_DOUBLE_EQ(m.nodes[0].y, 2.0);
}

TEST_F(ReadInput, PlacesOutOfOrderNodesById) {
    auto path = dir_ / "in.txt";
    write_file(path, "*NODES\n3 30.0 3.0\n1 10.0 1.0\n2 20.0 2.0\n");
    model m = read_input(path.string());
    ASSERT_EQ(m.nodes.size(), 3u);
    EXPECT_DOUBLE_EQ(m.nodes[0].x, 10.0);
    EXPECT_DOUBLE_EQ(m.nodes[1].x, 20.0);
    EXPECT_DOUBLE_EQ(m.nodes[2].x, 30.0);
}

TEST_F(ReadInput, IgnoresElementIdsAndKeepsFileOrder) {
    auto path = dir_ / "in.txt";
    write_file(path, "*ELEMENTS\n9 1 2 1.0 5.0\n4 2 3 2.0 6.0\n");
    model m = read_input(path.string());
    ASSERT_EQ(m.elems.size(), 2u);
    EXPECT_EQ(m.elems[0].n1, 1);
    EXPECT_DOUBLE_EQ(m.elems[0].e, 5.0);
    EXPECT_EQ(m.elems[1].n1, 2);
    EXPECT_DOUBLE_EQ(m.elems[1].a, 2.0);
}

TEST_F(ReadInput, SectionHeadersAreCaseSensitive) {
    auto path = dir_ / "in.txt";
    write_file(path, "*nodes\n1 0.0 0.0\n");
    model m = read_input(path.string());
    EXPECT_TRUE(m.nodes.empty());
}

TEST_F(ReadInput, LoadStepsDefaultsToOne) {
    auto path = dir_ / "in.txt";
    write_file(path, "*NODES\n1 0.0 0.0\n");
    EXPECT_EQ(read_input(path.string()).load_steps, 1);
}

TEST_F(ReadInput, ThrowsOnMissingFile) {
    EXPECT_THROW(read_input((dir_ / "does_not_exist.txt").string()),
                 std::runtime_error);
}
