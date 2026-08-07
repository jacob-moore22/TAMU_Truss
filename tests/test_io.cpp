#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "io.h"

TEST(ReadInput, ParsesAllSections) {
    model m = read_input(std::string(TEST_FIXTURES_DIR) + "/single_element.txt");

    ASSERT_EQ(m.nodes.size(), 2u);
    EXPECT_DOUBLE_EQ(m.nodes[0].x, 0.0);
    EXPECT_DOUBLE_EQ(m.nodes[0].y, 0.0);
    EXPECT_DOUBLE_EQ(m.nodes[1].x, 10.0);
    EXPECT_DOUBLE_EQ(m.nodes[1].y, 0.0);

    ASSERT_EQ(m.elems.size(), 1u);
    EXPECT_EQ(m.elems[0].n1, 1);
    EXPECT_EQ(m.elems[0].n2, 2);
    EXPECT_DOUBLE_EQ(m.elems[0].a, 1.0);
    EXPECT_DOUBLE_EQ(m.elems[0].e, 100.0);

    ASSERT_EQ(m.bcs.size(), 3u);
    EXPECT_EQ(m.bcs[0].node, 1);
    EXPECT_EQ(m.bcs[0].dof, 1);
    EXPECT_DOUBLE_EQ(m.bcs[0].val, 0.0);

    ASSERT_EQ(m.forces.size(), 1u);
    EXPECT_EQ(m.forces[0].node, 2);
    EXPECT_EQ(m.forces[0].dof, 1);
    EXPECT_DOUBLE_EQ(m.forces[0].val, 100.0);

    EXPECT_EQ(m.load_steps, 1);
}

TEST(ReadInput, ThrowsOnMissingFile) { EXPECT_THROW(read_input("this/path/does/not/exist.txt"), std::runtime_error); }

TEST(ReadInput, SkipsCommentsAndBlankLinesAndHandlesSparseNodeIds) {
    // fixture declares node 3 before node 2, with blank lines and comments interleaved
    model m = read_input(std::string(TEST_FIXTURES_DIR) + "/sparse_and_comments.txt");

    ASSERT_EQ(m.nodes.size(), 3u);
    EXPECT_DOUBLE_EQ(m.nodes[0].x, 0.0);  // node 1
    EXPECT_DOUBLE_EQ(m.nodes[1].x, 10.0); // node 2, filled in after the resize triggered by node 3
    EXPECT_DOUBLE_EQ(m.nodes[2].x, 5.0);  // node 3
    EXPECT_DOUBLE_EQ(m.nodes[2].y, 5.0);
}

TEST(ReadInput, DefaultsLoadStepsWhenSectionMissing) {
    // fixture has no *LOAD_STEPS section at all
    model m = read_input(std::string(TEST_FIXTURES_DIR) + "/sparse_and_comments.txt");

    EXPECT_EQ(m.load_steps, 1);
}
