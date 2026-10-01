#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "io.h"
#include "solver.h"
#include "test_helpers.h"

namespace {

constexpr double kE = 200e9;
constexpr double kA = 1.5;

// Absolute tolerance scaled to the magnitude being compared.
double tol(double scale) { return 1e-9 * std::max(1.0, std::fabs(scale)); }

}  // namespace

// ---- k_local ----

TEST(KLocal, HorizontalBarStiffnessOnlyInX) {
    auto ke = k_local({0, 0}, {2, 0}, kA, kE);
    double k = kE * kA / 2.0;
    EXPECT_NEAR(ke[0][0], k, tol(k));
    EXPECT_NEAR(ke[0][2], -k, tol(k));
    EXPECT_NEAR(ke[2][2], k, tol(k));
    EXPECT_NEAR(ke[1][1], 0.0, tol(k));
    EXPECT_NEAR(ke[3][3], 0.0, tol(k));
    EXPECT_NEAR(ke[0][1], 0.0, tol(k));
}

TEST(KLocal, VerticalBarStiffnessOnlyInY) {
    auto ke = k_local({0, 0}, {0, 4}, kA, kE);
    double k = kE * kA / 4.0;
    EXPECT_NEAR(ke[1][1], k, tol(k));
    EXPECT_NEAR(ke[1][3], -k, tol(k));
    EXPECT_NEAR(ke[3][3], k, tol(k));
    EXPECT_NEAR(ke[0][0], 0.0, tol(k));
    EXPECT_NEAR(ke[2][2], 0.0, tol(k));
}

TEST(KLocal, DiagonalBarSplitsStiffnessEvenly) {
    auto ke = k_local({0, 0}, {3, 3}, kA, kE);
    double l = std::sqrt(18.0);
    double half = kE * kA / l / 2.0;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            EXPECT_NEAR(ke[i][j], half, tol(half));
            EXPECT_NEAR(ke[i][j + 2], -half, tol(half));
        }
    }
}

TEST(KLocal, SymmetricWithZeroRowSums) {
    auto ke = k_local({1, 2}, {4, -2}, kA, kE);
    for (int i = 0; i < 4; ++i) {
        double row = 0.0;
        for (int j = 0; j < 4; ++j) {
            EXPECT_NEAR(ke[i][j], ke[j][i], tol(ke[i][j]));
            row += ke[i][j];
        }
        EXPECT_NEAR(row, 0.0, tol(ke[i][i]));
    }
}

// ---- assemble ----

TEST(Assemble, SizeIsTwiceNodeCount) {
    std::vector<node> nodes = {{0, 0}, {1, 0}, {0, 1}};
    matrix k = assemble(nodes, {{1, 2, kA, kE}});
    ASSERT_EQ(k.size(), 6u);
    for (const auto& row : k) EXPECT_EQ(row.size(), 6u);
}

TEST(Assemble, SingleElementMatchesKLocal) {
    std::vector<node> nodes = {{0, 0}, {3, 4}};
    matrix k = assemble(nodes, {{1, 2, kA, kE}});
    auto ke = k_local(nodes[0], nodes[1], kA, kE);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) EXPECT_DOUBLE_EQ(k[i][j], ke[i][j]);
}

TEST(Assemble, SharedNodeAccumulatesAndIsSymmetric) {
    // Two collinear bars meeting at node 2.
    std::vector<node> nodes = {{0, 0}, {1, 0}, {3, 0}};
    std::vector<elem> elems = {{1, 2, kA, kE}, {2, 3, kA, kE}};
    matrix k = assemble(nodes, elems);
    double k1 = kE * kA / 1.0;
    double k2 = kE * kA / 2.0;
    EXPECT_NEAR(k[2][2], k1 + k2, tol(k1));
    EXPECT_NEAR(k[0][4], 0.0, tol(k1));  // nodes 1 and 3 are not connected
    for (size_t i = 0; i < k.size(); ++i)
        for (size_t j = 0; j < k.size(); ++j)
            EXPECT_DOUBLE_EQ(k[i][j], k[j][i]);
}

// ---- build_f ----

TEST(BuildF, PlacesForcesAtGlobalDofs) {
    auto f = build_f({{2, 1, 7.0}, {3, 2, -4.0}}, 6);
    ASSERT_EQ(f.size(), 6u);
    EXPECT_DOUBLE_EQ(f[2], 7.0);
    EXPECT_DOUBLE_EQ(f[5], -4.0);
    EXPECT_DOUBLE_EQ(f[0], 0.0);
    EXPECT_DOUBLE_EQ(f[1], 0.0);
    EXPECT_DOUBLE_EQ(f[3], 0.0);
    EXPECT_DOUBLE_EQ(f[4], 0.0);
}

TEST(BuildF, RepeatedForcesOnSameDofAdd) {
    auto f = build_f({{1, 2, 3.0}, {1, 2, 4.5}}, 2);
    EXPECT_DOUBLE_EQ(f[1], 7.5);
}

// ---- apply_bc ----

TEST(ApplyBc, ZeroesRowAndColumnWithUnitDiagonal) {
    matrix k = {{4, 1, 2, 3}, {1, 5, 6, 7}, {2, 6, 8, 9}, {3, 7, 9, 10}};
    std::vector<double> f = {1, 2, 3, 4};
    apply_bc(k, f, {{1, 2, 0.0}});  // global dof 1
    for (int i = 0; i < 4; ++i) {
        if (i == 1) continue;
        EXPECT_DOUBLE_EQ(k[1][i], 0.0);
        EXPECT_DOUBLE_EQ(k[i][1], 0.0);
    }
    EXPECT_DOUBLE_EQ(k[1][1], 1.0);
    EXPECT_DOUBLE_EQ(k[0][0], 4.0);  // untouched
    EXPECT_DOUBLE_EQ(f[1], 0.0);
    EXPECT_DOUBLE_EQ(f[0], 1.0);  // zero prescribed value: rhs unchanged
}

TEST(ApplyBc, NonZeroPrescribedDisplacementMovesToRhs) {
    matrix k = {{2, -1}, {-1, 2}};
    std::vector<double> f = {0, 0};
    apply_bc(k, f, {{1, 1, 0.5}});
    EXPECT_DOUBLE_EQ(f[0], 0.5);
    EXPECT_DOUBLE_EQ(f[1], 0.5);  // 0 - (-1)*0.5
    auto u = gauss_solve(k, f);
    EXPECT_NEAR(u[0], 0.5, 1e-12);
    EXPECT_NEAR(u[1], 0.25, 1e-12);
}

// ---- gauss_solve ----

TEST(GaussSolve, Solves2x2) {
    auto u = gauss_solve({{2, 1}, {1, 3}}, {3, 5});
    EXPECT_NEAR(u[0], 0.8, 1e-12);
    EXPECT_NEAR(u[1], 1.4, 1e-12);
}

TEST(GaussSolve, Solves3x3) {
    auto u = gauss_solve({{4, -2, 1}, {-2, 4, -2}, {1, -2, 4}}, {11, -16, 17});
    EXPECT_NEAR(u[0], 1.0, 1e-12);
    EXPECT_NEAR(u[1], -2.0, 1e-12);
    EXPECT_NEAR(u[2], 3.0, 1e-12);
}

TEST(GaussSolve, PivotsAroundZeroDiagonal) {
    auto u = gauss_solve({{0, 1}, {1, 0}}, {2, 3});
    EXPECT_NEAR(u[0], 3.0, 1e-12);
    EXPECT_NEAR(u[1], 2.0, 1e-12);
}

TEST(GaussSolve, ThrowsOnSingularMatrix) {
    EXPECT_THROW(gauss_solve({{1, 2}, {2, 4}}, {1, 2}), std::runtime_error);
}

TEST(GaussSolve, ThrowsOnUnconstrainedTruss) {
    std::vector<node> nodes = {{0, 0}, {1, 0}};
    matrix k = assemble(nodes, {{1, 2, kA, kE}});
    EXPECT_THROW(gauss_solve(k, std::vector<double>(4, 0.0)),
                 std::runtime_error);
}

TEST(GaussSolve, DoesNotModifyCallerInputs) {
    matrix k = {{0, 1}, {1, 0}};
    std::vector<double> f = {2, 3};
    gauss_solve(k, f);
    EXPECT_EQ(k, (matrix{{0, 1}, {1, 0}}));
    EXPECT_EQ(f, (std::vector<double>{2, 3}));
}

// ---- reactions ----

TEST(Reactions, SingleBarReactionBalancesLoad) {
    // Bar along x, node 1 pinned, node 2 on a y-roller, pulled with P in x.
    std::vector<node> nodes = {{0, 0}, {2, 0}};
    std::vector<elem> elems = {{1, 2, kA, kE}};
    std::vector<bc> bcs = {{1, 1, 0}, {1, 2, 0}, {2, 2, 0}};
    double p = 1000.0;
    matrix k = assemble(nodes, elems);
    auto f = build_f({{2, 1, p}}, 4);
    matrix k_bc = k;
    auto f_bc = f;
    apply_bc(k_bc, f_bc, bcs);
    auto u = gauss_solve(k_bc, f_bc);
    EXPECT_NEAR(u[2], p * 2.0 / (kE * kA), 1e-15);

    auto r = reactions(k, u, f);
    EXPECT_NEAR(r[0], -p, tol(p));
    EXPECT_NEAR(r[2], 0.0, tol(p));  // free dof
    EXPECT_NEAR(r[1], 0.0, tol(p));
    EXPECT_NEAR(r[3], 0.0, tol(p));
}

// ---- elem_stress ----

TEST(ElemStress, AxialStretchGivesEDeltaOverL) {
    std::vector<node> nodes = {{0, 0}, {4, 0}};
    double delta = 1e-3;
    auto s = elem_stress(nodes, {{1, 2, kA, kE}}, {0, 0, delta, 0});
    ASSERT_EQ(s.size(), 1u);
    EXPECT_NEAR(s[0], kE * delta / 4.0, tol(kE * delta));
}

TEST(ElemStress, CompressionIsNegative) {
    std::vector<node> nodes = {{0, 0}, {3, 4}};
    // Move node 2 toward node 1 along the bar axis by 0.01.
    std::vector<double> u = {0, 0, -0.006, -0.008};
    auto s = elem_stress(nodes, {{1, 2, kA, kE}}, u);
    EXPECT_NEAR(s[0], -kE * 0.01 / 5.0, tol(kE * 0.01));
}

TEST(ElemStress, RigidBodyMotionGivesZeroStress) {
    std::vector<node> nodes = {{1, 2}, {4, 6}};
    std::vector<elem> elems = {{1, 2, kA, kE}};
    // Translation.
    auto s1 = elem_stress(nodes, elems, {0.3, -0.2, 0.3, -0.2});
    EXPECT_NEAR(s1[0], 0.0, 1e-3);
    // Infinitesimal rotation u = theta * (-y, x).
    double th = 1e-4;
    auto s2 = elem_stress(nodes, elems, {-th * 2, th * 1, -th * 6, th * 4});
    EXPECT_NEAR(s2[0], 0.0, 1e-3);
}

// ---- integration: triangle example vs. method of joints ----

TEST(SolverIntegration, TriangleMatchesHandCalculation) {
    model m = read_input(example_path("input_triangle.txt"));
    int n_dof = 2 * (int)m.nodes.size();
    matrix k = assemble(m.nodes, m.elems);
    auto f = build_f(m.forces, n_dof);
    matrix k_bc = k;
    auto f_bc = f;
    apply_bc(k_bc, f_bc, m.bcs);
    auto u = gauss_solve(k_bc, f_bc);
    auto r = reactions(k, u, f);
    auto s = elem_stress(m.nodes, m.elems, u);

    // Supports: node 1 pinned, node 2 roller in y. Load (5000, -10000) at 3.
    EXPECT_NEAR(r[0], -5000.0, 1e-3);
    EXPECT_NEAR(r[1], 0.0, 1e-3);
    EXPECT_NEAR(r[3], 10000.0, 1e-3);
    EXPECT_NEAR(r[0] + 5000.0, 0.0, 1e-3);
    EXPECT_NEAR(r[1] + r[3] - 10000.0, 0.0, 1e-3);

    // Member forces: 1-2 = +5000, 2-3 = -1000*sqrt(125), 1-3 = 0.
    EXPECT_NEAR(s[0], 5000.0 / 1.5, 1e-3);
    EXPECT_NEAR(s[1], -1000.0 * std::sqrt(125.0) / 1.5, 1e-3);
    EXPECT_NEAR(s[2], 0.0, 1e-3);
}
