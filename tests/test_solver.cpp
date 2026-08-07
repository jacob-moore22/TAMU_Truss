#include <gtest/gtest.h>

#include <cmath>

#include "solver.h"

namespace {
constexpr double kTol = 1e-9;
}

// ---------------------------------------------------------------------------
// k_local
// ---------------------------------------------------------------------------

TEST(KLocal, HorizontalElementIsAxialOnly) {
    node a{0.0, 0.0};
    node b{5.0, 0.0};
    auto k = k_local(a, b, /*cross_section_area=*/2.0, /*youngs_modulus=*/100.0);
    // k = E*A/L = 100*2/5 = 40
    double expected[4][4] = {
        {40, 0, -40, 0},
        {0, 0, 0, 0},
        {-40, 0, 40, 0},
        {0, 0, 0, 0},
    };
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(k[i][j], expected[i][j], kTol) << "at (" << i << "," << j << ")";
}

TEST(KLocal, VerticalElementIsAxialOnly) {
    node a{0.0, 0.0};
    node b{0.0, 5.0};
    auto k = k_local(a, b, 2.0, 100.0);
    double expected[4][4] = {
        {0, 0, 0, 0},
        {0, 40, 0, -40},
        {0, 0, 0, 0},
        {0, -40, 0, 40},
    };
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(k[i][j], expected[i][j], kTol) << "at (" << i << "," << j << ")";
}

TEST(KLocal, DiagonalElementHasCouplingTerms) {
    // 3-4-5 triangle: dx=3, dy=4, length=5, cos=0.6, sin=0.8.
    node a{0.0, 0.0};
    node b{3.0, 4.0};
    auto k = k_local(a, b, 2.0, 100.0);
    // axial_stiffness = E*A/L = 40
    double expected[4][4] = {
        {14.4, 19.2, -14.4, -19.2},
        {19.2, 25.6, -19.2, -25.6},
        {-14.4, -19.2, 14.4, 19.2},
        {-19.2, -25.6, 19.2, 25.6},
    };
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(k[i][j], expected[i][j], kTol) << "at (" << i << "," << j << ")";
}

// ---------------------------------------------------------------------------
// assemble
// ---------------------------------------------------------------------------

TEST(Assemble, TwoElementChainOverlapsAtSharedNode) {
    // Three collinear nodes, two horizontal elements, matching the k_local
    // horizontal case (k = 40 each).
    std::vector<node> nodes = {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}};
    std::vector<elem> elements = {
        {1, 2, 2.0, 100.0},
        {2, 3, 2.0, 100.0},
    };
    matrix k = assemble(nodes, elements);

    ASSERT_EQ(k.size(), 6u);
    for (const auto& row : k) ASSERT_EQ(row.size(), 6u);

    // Node 1 (dofs 0,1): only touched by element 1.
    EXPECT_NEAR(k[0][0], 40.0, kTol);
    EXPECT_NEAR(k[0][2], -40.0, kTol);
    EXPECT_NEAR(k[0][4], 0.0, kTol);

    // Node 2 (dofs 2,3): shared, contributions from both elements sum.
    EXPECT_NEAR(k[2][2], 80.0, kTol);
    EXPECT_NEAR(k[2][0], -40.0, kTol);
    EXPECT_NEAR(k[2][4], -40.0, kTol);

    // Node 3 (dofs 4,5): only touched by element 2.
    EXPECT_NEAR(k[4][4], 40.0, kTol);
    EXPECT_NEAR(k[4][2], -40.0, kTol);

    // Horizontal elements contribute nothing to the y-y block.
    EXPECT_NEAR(k[1][1], 0.0, kTol);
    EXPECT_NEAR(k[3][3], 0.0, kTol);
    EXPECT_NEAR(k[5][5], 0.0, kTol);
}

// ---------------------------------------------------------------------------
// build_f
// ---------------------------------------------------------------------------

TEST(BuildF, AccumulatesForcesOnSameDofAndIsolatesOthers) {
    std::vector<force> forces = {
        {1, 1, 100.0},  // dof 0
        {1, 1, 50.0},   // dof 0 again -> accumulates
        {2, 2, -30.0},  // dof 3
    };
    auto f = build_f(forces, /*dof_count=*/4);
    ASSERT_EQ(f.size(), 4u);
    EXPECT_NEAR(f[0], 150.0, kTol);
    EXPECT_NEAR(f[1], 0.0, kTol);
    EXPECT_NEAR(f[2], 0.0, kTol);
    EXPECT_NEAR(f[3], -30.0, kTol);
}

// ---------------------------------------------------------------------------
// apply_bc
// ---------------------------------------------------------------------------

TEST(ApplyBc, EliminatesConstrainedDofWithNonzeroPrescribedValue) {
    matrix k = {
        {10, 1, 2, 3},
        {1, 10, 4, 5},
        {2, 4, 10, 6},
        {3, 5, 6, 10},
    };
    std::vector<double> f = {1, 2, 3, 4};
    std::vector<bc> bcs = {{1, 1, 0.5}};  // node 1, dof 1 -> dof index 0, value 0.5

    apply_bc(k, f, bcs);

    // Constrained row/column zeroed, diagonal set to 1.
    EXPECT_NEAR(k[0][0], 1.0, kTol);
    EXPECT_NEAR(k[0][1], 0.0, kTol);
    EXPECT_NEAR(k[0][2], 0.0, kTol);
    EXPECT_NEAR(k[0][3], 0.0, kTol);
    EXPECT_NEAR(k[1][0], 0.0, kTol);
    EXPECT_NEAR(k[2][0], 0.0, kTol);
    EXPECT_NEAR(k[3][0], 0.0, kTol);

    // Unconstrained block is untouched.
    EXPECT_NEAR(k[1][1], 10.0, kTol);
    EXPECT_NEAR(k[1][2], 4.0, kTol);
    EXPECT_NEAR(k[2][3], 6.0, kTol);
    EXPECT_NEAR(k[3][3], 10.0, kTol);

    // RHS: prescribed dof set to the BC value; others corrected by
    // f[i] -= K[i][dof] * value using the ORIGINAL column.
    EXPECT_NEAR(f[0], 0.5, kTol);
    EXPECT_NEAR(f[1], 2 - 1 * 0.5, kTol);
    EXPECT_NEAR(f[2], 3 - 2 * 0.5, kTol);
    EXPECT_NEAR(f[3], 4 - 3 * 0.5, kTol);
}

// ---------------------------------------------------------------------------
// gauss_solve
// ---------------------------------------------------------------------------

TEST(GaussSolve, SolvesSimpleTwoByTwoSystem) {
    matrix k = {{2, 1}, {1, 3}};
    std::vector<double> f = {3, 4};  // K * [1,1]^T = [3,4]^T
    auto u = gauss_solve(k, f);
    ASSERT_EQ(u.size(), 2u);
    EXPECT_NEAR(u[0], 1.0, kTol);
    EXPECT_NEAR(u[1], 1.0, kTol);
}

TEST(GaussSolve, RequiresPartialPivotingOnZeroDiagonal) {
    matrix k = {{0, 1}, {1, 0}};
    std::vector<double> f = {3, 2};
    auto u = gauss_solve(k, f);
    ASSERT_EQ(u.size(), 2u);
    EXPECT_NEAR(u[0], 2.0, kTol);
    EXPECT_NEAR(u[1], 3.0, kTol);
}

TEST(GaussSolve, ThrowsOnSingularMatrix) {
    matrix k = {{1, 1}, {1, 1}};
    std::vector<double> f = {1, 1};
    EXPECT_THROW(gauss_solve(k, f), std::runtime_error);
}

// ---------------------------------------------------------------------------
// reactions
// ---------------------------------------------------------------------------

TEST(Reactions, ComputesKUMinusF) {
    matrix k = {{2, 1}, {1, 3}};
    std::vector<double> u = {1, 1};
    std::vector<double> f = {1, 2};
    // K*u = [3,4]; reactions = K*u - f = [2,2]
    auto r = reactions(k, u, f);
    ASSERT_EQ(r.size(), 2u);
    EXPECT_NEAR(r[0], 2.0, kTol);
    EXPECT_NEAR(r[1], 2.0, kTol);
}

// ---------------------------------------------------------------------------
// elem_stress
// ---------------------------------------------------------------------------

TEST(ElemStress, PureTensionIsPositive) {
    std::vector<node> nodes = {{0.0, 0.0}, {5.0, 0.0}};
    std::vector<elem> elements = {{1, 2, 2.0, 100.0}};
    std::vector<double> displacements = {0.0, 0.0, 0.01, 0.0};  // node2 stretches +x by 0.01
    auto s = elem_stress(nodes, elements, displacements);
    ASSERT_EQ(s.size(), 1u);
    // strain = 0.01/5 = 0.002; stress = E*strain = 100*0.002 = 0.2
    EXPECT_NEAR(s[0], 0.2, kTol);
}

TEST(ElemStress, PureCompressionIsNegative) {
    std::vector<node> nodes = {{0.0, 0.0}, {5.0, 0.0}};
    std::vector<elem> elements = {{1, 2, 2.0, 100.0}};
    std::vector<double> displacements = {0.0, 0.0, -0.01, 0.0};  // node2 moves -x by 0.01
    auto s = elem_stress(nodes, elements, displacements);
    ASSERT_EQ(s.size(), 1u);
    EXPECT_NEAR(s[0], -0.2, kTol);
}
