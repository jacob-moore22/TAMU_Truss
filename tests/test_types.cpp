#include <gtest/gtest.h>

#include "types.h"

TEST(Types, ModelDefaultsToOneLoadStep) {
    model m;
    EXPECT_EQ(m.load_steps, 1);
    EXPECT_TRUE(m.nodes.empty());
    EXPECT_TRUE(m.elems.empty());
    EXPECT_TRUE(m.bcs.empty());
    EXPECT_TRUE(m.forces.empty());
}

// The parser and the tests rely on aggregate init in this field order.
TEST(Types, AggregateInitFieldOrder) {
    node n{1.0, 2.0};
    EXPECT_DOUBLE_EQ(n.x, 1.0);
    EXPECT_DOUBLE_EQ(n.y, 2.0);

    elem e{3, 4, 1.5, 200e9};
    EXPECT_EQ(e.n1, 3);
    EXPECT_EQ(e.n2, 4);
    EXPECT_DOUBLE_EQ(e.a, 1.5);
    EXPECT_DOUBLE_EQ(e.e, 200e9);

    bc b{2, 1, -0.5};
    EXPECT_EQ(b.node, 2);
    EXPECT_EQ(b.dof, 1);
    EXPECT_DOUBLE_EQ(b.val, -0.5);

    force f{5, 2, 100.0};
    EXPECT_EQ(f.node, 5);
    EXPECT_EQ(f.dof, 2);
    EXPECT_DOUBLE_EQ(f.val, 100.0);
}
