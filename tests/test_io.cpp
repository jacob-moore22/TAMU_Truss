#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "io.h"
#include "test_helpers.h"

namespace {
const std::string kExamples = TRUSS_EXAMPLES_DIR;
}

TEST(ReadInput, TriangleExample) {
    model m = read_input(kExamples + "/input_triangle.txt");

    ASSERT_EQ(m.nodes.size(), 3u);
    EXPECT_DOUBLE_EQ(m.nodes[0].x, 0.0);
    EXPECT_DOUBLE_EQ(m.nodes[0].y, 0.0);
    EXPECT_DOUBLE_EQ(m.nodes[1].x, 10.0);
    EXPECT_DOUBLE_EQ(m.nodes[1].y, 0.0);
    EXPECT_DOUBLE_EQ(m.nodes[2].x, 5.0);
    EXPECT_DOUBLE_EQ(m.nodes[2].y, 10.0);

    ASSERT_EQ(m.elements.size(), 3u);
    EXPECT_EQ(m.elements[0].node_1, 1);
    EXPECT_EQ(m.elements[0].node_2, 2);
    EXPECT_EQ(m.elements[1].node_1, 2);
    EXPECT_EQ(m.elements[1].node_2, 3);
    EXPECT_EQ(m.elements[2].node_1, 1);
    EXPECT_EQ(m.elements[2].node_2, 3);
    for (const auto& e : m.elements) {
        EXPECT_DOUBLE_EQ(e.area, 1.5);
        EXPECT_DOUBLE_EQ(e.youngs_modulus, 200e9);
    }

    ASSERT_EQ(m.supports.size(), 3u);
    EXPECT_EQ(m.supports[0].node_id, 1);
    EXPECT_EQ(m.supports[0].dof, 1);
    EXPECT_EQ(m.supports[1].node_id, 1);
    EXPECT_EQ(m.supports[1].dof, 2);
    EXPECT_EQ(m.supports[2].node_id, 2);
    EXPECT_EQ(m.supports[2].dof, 2);
    for (const auto& b : m.supports) EXPECT_DOUBLE_EQ(b.value, 0.0);

    ASSERT_EQ(m.loads.size(), 2u);
    EXPECT_EQ(m.loads[0].node_id, 3);
    EXPECT_EQ(m.loads[0].dof, 1);
    EXPECT_DOUBLE_EQ(m.loads[0].value, 5000.0);
    EXPECT_EQ(m.loads[1].node_id, 3);
    EXPECT_EQ(m.loads[1].dof, 2);
    EXPECT_DOUBLE_EQ(m.loads[1].value, -10000.0);

    EXPECT_EQ(m.num_load_steps, 10);
}

TEST(ReadInput, FinkAndHoweExamplesAreConsistent) {
    for (const char* name : {"fink_truss.txt", "howe_truss.txt"}) {
        SCOPED_TRACE(name);
        model m = read_input(kExamples + "/" + name);
        EXPECT_GT(m.nodes.size(), 3u);
        EXPECT_GT(m.elements.size(), m.nodes.size());
        EXPECT_GE(m.supports.size(), 3u);
        EXPECT_FALSE(m.loads.empty());
        EXPECT_GT(m.num_load_steps, 0);
        int n = (int)m.nodes.size();
        for (const auto& e : m.elements) {
            EXPECT_GE(e.node_1, 1);
            EXPECT_LE(e.node_1, n);
            EXPECT_GE(e.node_2, 1);
            EXPECT_LE(e.node_2, n);
            EXPECT_NE(e.node_1, e.node_2);
            EXPECT_GT(e.area, 0.0);
            EXPECT_GT(e.youngs_modulus, 0.0);
        }
        for (const auto& b : m.supports) {
            EXPECT_GE(b.node_id, 1);
            EXPECT_LE(b.node_id, n);
            EXPECT_TRUE(b.dof == 1 || b.dof == 2);
        }
        for (const auto& f : m.loads) {
            EXPECT_GE(f.node_id, 1);
            EXPECT_LE(f.node_id, n);
            EXPECT_TRUE(f.dof == 1 || f.dof == 2);
        }
    }
}

TEST(ReadInput, MissingFileThrows) {
    EXPECT_THROW(read_input("/nonexistent/dir/input.txt"), std::runtime_error);
}

TEST(ReadInput, IgnoresCommentsAndBlankLines) {
    temp_dir d;
    std::string p = d.write_file("in.txt",
                                 "# leading comment\n"
                                 "\n"
                                 "*NODES\n"
                                 "# ID X Y\n"
                                 "1 0 0\n"
                                 "\n"
                                 "2 1 0\n"
                                 "   # indented comment\n"
                                 "*ELEMENTS\n"
                                 "1 1 2 1 1\n"
                                 "*LOAD_STEPS\n"
                                 "# N\n"
                                 "3\n");
    model m = read_input(p);
    ASSERT_EQ(m.nodes.size(), 2u);
    ASSERT_EQ(m.elements.size(), 1u);
    EXPECT_EQ(m.num_load_steps, 3);
}

TEST(ReadInput, SectionsInAnyOrder) {
    temp_dir d;
    std::string p = d.write_file("in.txt",
                                 "*LOAD_STEPS\n5\n"
                                 "*FORCES\n2 1 7\n"
                                 "*BOUNDARIES\n1 1 0\n1 2 0\n"
                                 "*ELEMENTS\n1 1 2 1 1\n"
                                 "*NODES\n1 0 0\n2 1 0\n");
    model m = read_input(p);
    EXPECT_EQ(m.num_load_steps, 5);
    ASSERT_EQ(m.loads.size(), 1u);
    EXPECT_DOUBLE_EQ(m.loads[0].value, 7.0);
    EXPECT_EQ(m.supports.size(), 2u);
    EXPECT_EQ(m.elements.size(), 1u);
    EXPECT_EQ(m.nodes.size(), 2u);
}

TEST(ReadInput, NodesOutOfOrderLandInIdSlot) {
    temp_dir d;
    std::string p = d.write_file("in.txt",
                                 "*NODES\n"
                                 "3 30 33\n"
                                 "1 10 11\n"
                                 "2 20 22\n");
    model m = read_input(p);
    ASSERT_EQ(m.nodes.size(), 3u);
    EXPECT_DOUBLE_EQ(m.nodes[0].x, 10.0);
    EXPECT_DOUBLE_EQ(m.nodes[1].x, 20.0);
    EXPECT_DOUBLE_EQ(m.nodes[2].x, 30.0);
    EXPECT_DOUBLE_EQ(m.nodes[2].y, 33.0);
}

TEST(ReadInput, MissingLoadStepsDefaultsToOne) {
    temp_dir d;
    std::string p = d.write_file("in.txt", "*NODES\n1 0 0\n");
    model m = read_input(p);
    EXPECT_EQ(m.num_load_steps, 1);
}

TEST(ReadInput, HeadersOnlyGivesEmptyModel) {
    temp_dir d;
    std::string p = d.write_file(
        "in.txt", "*NODES\n*ELEMENTS\n*BOUNDARIES\n*FORCES\n*LOAD_STEPS\n");
    model m = read_input(p);
    EXPECT_TRUE(m.nodes.empty());
    EXPECT_TRUE(m.elements.empty());
    EXPECT_TRUE(m.supports.empty());
    EXPECT_TRUE(m.loads.empty());
    EXPECT_EQ(m.num_load_steps, 1);
}

TEST(ReadInput, EmptyFileGivesEmptyModel) {
    temp_dir d;
    std::string p = d.write_file("in.txt", "");
    model m = read_input(p);
    EXPECT_TRUE(m.nodes.empty());
    EXPECT_EQ(m.num_load_steps, 1);
}

TEST(ReadInput, DataBeforeAnySectionIsIgnored) {
    temp_dir d;
    std::string p = d.write_file("in.txt",
                                 "1 0 0\n"
                                 "2 1 0\n"
                                 "*NODES\n"
                                 "1 5 5\n");
    model m = read_input(p);
    ASSERT_EQ(m.nodes.size(), 1u);
    EXPECT_DOUBLE_EQ(m.nodes[0].x, 5.0);
}

TEST(ReadInput, ElementIdColumnIsIgnored) {
    // Element rows carry an ID column that the parser reads and discards;
    // elements are stored in file order regardless of that ID.
    temp_dir d;
    std::string p = d.write_file("in.txt",
                                 "*ELEMENTS\n"
                                 "9 1 2 1 1\n"
                                 "4 2 3 2 2\n");
    model m = read_input(p);
    ASSERT_EQ(m.elements.size(), 2u);
    EXPECT_EQ(m.elements[0].node_1, 1);
    EXPECT_EQ(m.elements[1].node_1, 2);
    EXPECT_DOUBLE_EQ(m.elements[1].area, 2.0);
}

// Characterization test: this documents CURRENT behaviour, not a spec.
// TODO(io): the parser does not validate numeric fields. A node line with a
// non-numeric coordinate leaves the stream in a failed state and the node is
// stored with whatever was parsed before the failure (zero-initialized by
// operator>> on failure in C++11+). Consider rejecting malformed lines.
TEST(ReadInput, MalformedNumericLineIsSilentlyAccepted) {
    temp_dir d;
    std::string p = d.write_file("in.txt",
                                 "*NODES\n"
                                 "1 abc 2.0\n");
    model m;
    EXPECT_NO_THROW(m = read_input(p));
    ASSERT_EQ(m.nodes.size(), 1u);
    EXPECT_DOUBLE_EQ(m.nodes[0].x, 0.0);
}
