#include <gtest/gtest.h>

#include "types.h"

TEST(Types, ModelDefaultsToOneLoadStep) {
    model m;
    EXPECT_EQ(m.num_load_steps, 1);
    EXPECT_TRUE(m.nodes.empty());
    EXPECT_TRUE(m.elements.empty());
    EXPECT_TRUE(m.supports.empty());
    EXPECT_TRUE(m.loads.empty());
}

// The parser and the tests rely on aggregate init in this field order.
TEST(Types, AggregateInitFieldOrder) {
    node n{1.0, 2.0};
    EXPECT_DOUBLE_EQ(n.x, 1.0);
    EXPECT_DOUBLE_EQ(n.y, 2.0);

    element e{3, 4, 1.5, 200e9};
    EXPECT_EQ(e.node_1, 3);
    EXPECT_EQ(e.node_2, 4);
    EXPECT_DOUBLE_EQ(e.area, 1.5);
    EXPECT_DOUBLE_EQ(e.youngs_modulus, 200e9);

    boundary_condition b{2, 1, -0.5};
    EXPECT_EQ(b.node_id, 2);
    EXPECT_EQ(b.dof, 1);
    EXPECT_DOUBLE_EQ(b.value, -0.5);

    force f{5, 2, 100.0};
    EXPECT_EQ(f.node_id, 5);
    EXPECT_EQ(f.dof, 2);
    EXPECT_DOUBLE_EQ(f.value, 100.0);
}
