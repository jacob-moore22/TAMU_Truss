#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "io.h"
#include "solver.h"
#include "types.h"

namespace {
constexpr double kEps = 1e-9;
}

// --- k_local -----------------------------------------------------------

TEST(KLocal, HorizontalElement) {
    node a{0.0, 0.0};
    node b{10.0, 0.0};
    double area = 2.0, e = 100.0;
    double k = e * area / 10.0; // 20.0

    auto ke = k_local(a, b, area, e);

    EXPECT_NEAR(ke[0][0], k, kEps);
    EXPECT_NEAR(ke[0][2], -k, kEps);
    EXPECT_NEAR(ke[1][1], 0.0, kEps);
    EXPECT_NEAR(ke[1][3], 0.0, kEps);
    EXPECT_NEAR(ke[2][2], k, kEps);
}

TEST(KLocal, VerticalElement) {
    node a{0.0, 0.0};
    node b{0.0, 5.0};
    double area = 1.0, e = 50.0;
    double k = e * area / 5.0; // 10.0

    auto ke = k_local(a, b, area, e);

    EXPECT_NEAR(ke[0][0], 0.0, kEps);
    EXPECT_NEAR(ke[1][1], k, kEps);
    EXPECT_NEAR(ke[1][3], -k, kEps);
}

TEST(KLocal, FortyFiveDegreeElement) {
    node a{0.0, 0.0};
    node b{1.0, 1.0};
    double area = 1.0, e = 1.0;
    double l = std::sqrt(2.0);
    double k = e * area / l;
    double cs = 1.0 / std::sqrt(2.0);

    auto ke = k_local(a, b, area, e);

    EXPECT_NEAR(ke[0][0], k * cs * cs, kEps);
    EXPECT_NEAR(ke[0][1], k * cs * cs, kEps);
}

TEST(KLocal, MatrixIsSymmetric) {
    node a{1.0, 2.0};
    node b{7.0, -3.0};
    auto ke = k_local(a, b, 3.0, 70.0);

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            EXPECT_NEAR(ke[i][j], ke[j][i], kEps);
        }
    }
}

TEST(KLocal, ScalesLinearlyWithAreaAndModulus) {
    node a{0.0, 0.0};
    node b{4.0, 0.0};
    auto base = k_local(a, b, 1.0, 1.0);
    auto doubleArea = k_local(a, b, 2.0, 1.0);
    auto doubleE = k_local(a, b, 1.0, 2.0);

    EXPECT_NEAR(doubleArea[0][0], 2.0 * base[0][0], kEps);
    EXPECT_NEAR(doubleE[0][0], 2.0 * base[0][0], kEps);
}

// --- assemble ------------------------------------------------------------

TEST(Assemble, SingleElementMatchesKLocal) {
    std::vector<node> nodes = {{0.0, 0.0}, {10.0, 0.0}};
    std::vector<elem> elems = {{1, 2, 2.0, 100.0}};

    matrix kg = assemble(nodes, elems);
    auto ke = k_local(nodes[0], nodes[1], 2.0, 100.0);

    ASSERT_EQ(kg.size(), 4u);
    for (int i = 0; i < 4; ++i) {
        ASSERT_EQ(kg[i].size(), 4u);
        for (int j = 0; j < 4; ++j) {
            EXPECT_NEAR(kg[i][j], ke[i][j], kEps);
        }
    }
}

TEST(Assemble, TwoElementsSuperposeAtSharedNode) {
    std::vector<node> nodes = {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
    std::vector<elem> elems = {{1, 2, 1.0, 100.0}, {2, 3, 1.0, 100.0}};

    matrix kg = assemble(nodes, elems);
    auto ke1 = k_local(nodes[0], nodes[1], 1.0, 100.0);
    auto ke2 = k_local(nodes[1], nodes[2], 1.0, 100.0);

    ASSERT_EQ(kg.size(), 6u);
    // node 2 (dofs 2,3) is shared between element 1 (local 2,3) and element 2 (local 0,1)
    EXPECT_NEAR(kg[2][2], ke1[2][2] + ke2[0][0], kEps);
    EXPECT_NEAR(kg[3][3], ke1[3][3] + ke2[1][1], kEps);
    // untouched corners come from a single element only
    EXPECT_NEAR(kg[0][0], ke1[0][0], kEps);
    EXPECT_NEAR(kg[5][5], ke2[3][3], kEps);
}

TEST(Assemble, MatrixSizeMatchesDofCount) {
    std::vector<node> nodes = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
    std::vector<elem> elems = {{1, 2, 1.0, 1.0}, {2, 3, 1.0, 1.0}, {3, 4, 1.0, 1.0}};

    matrix kg = assemble(nodes, elems);

    ASSERT_EQ(kg.size(), 8u);
    for (const auto &row : kg) ASSERT_EQ(row.size(), 8u);
}

TEST(Assemble, MatrixIsSymmetric) {
    std::vector<node> nodes = {{0, 0}, {5, 5}, {10, 0}};
    std::vector<elem> elems = {{1, 2, 1.0, 100.0}, {2, 3, 1.5, 80.0}};

    matrix kg = assemble(nodes, elems);
    for (size_t i = 0; i < kg.size(); ++i) {
        for (size_t j = 0; j < kg.size(); ++j) {
            EXPECT_NEAR(kg[i][j], kg[j][i], kEps);
        }
    }
}

// --- build_f ---------------------------------------------------------------

TEST(BuildF, SingleForceMapsToCorrectDof) {
    std::vector<force> forces = {{2, 1, 500.0}}; // node 2, dof x -> index 2
    auto f = build_f(forces, 6);

    ASSERT_EQ(f.size(), 6u);
    EXPECT_NEAR(f[2], 500.0, kEps);
    for (int i = 0; i < 6; ++i) {
        if (i != 2) {
            EXPECT_NEAR(f[i], 0.0, kEps);
        }
    }
}

TEST(BuildF, MultipleForcesOnSameDofAccumulate) {
    std::vector<force> forces = {{1, 2, 100.0}, {1, 2, 50.0}}; // node1 dof y -> index 1
    auto f = build_f(forces, 4);

    EXPECT_NEAR(f[1], 150.0, kEps);
}

// --- apply_bc ----------------------------------------------------------------

TEST(ApplyBc, ZerosRowAndColSetsDiagonalOne) {
    matrix k = {{4.0, 1.0}, {1.0, 3.0}};
    std::vector<double> f = {10.0, 20.0};
    std::vector<bc> bcs = {{1, 1, 5.0}}; // node1 dof x -> index 0

    apply_bc(k, f, bcs);

    EXPECT_NEAR(k[0][0], 1.0, kEps);
    EXPECT_NEAR(k[0][1], 0.0, kEps);
    EXPECT_NEAR(k[1][0], 0.0, kEps);
    EXPECT_NEAR(k[1][1], 3.0, kEps); // untouched
    EXPECT_NEAR(f[0], 5.0, kEps);
    EXPECT_NEAR(f[1], 15.0, kEps); // 20 - k_orig[1][0]*5
}

TEST(ApplyBc, MultipleBcsAppliedInSequence) {
    matrix k = {{4.0, 1.0, 2.0}, {1.0, 5.0, 1.0}, {2.0, 1.0, 6.0}};
    std::vector<double> f = {10.0, 20.0, 30.0};
    std::vector<bc> bcs = {{1, 1, 0.0}, {2, 1, 0.0}}; // dof0 and dof2, both zero-valued

    apply_bc(k, f, bcs);

    EXPECT_NEAR(k[0][0], 1.0, kEps);
    EXPECT_NEAR(k[2][2], 1.0, kEps);
    EXPECT_NEAR(k[1][1], 5.0, kEps); // never touched
    EXPECT_NEAR(k[0][2], 0.0, kEps);
    EXPECT_NEAR(k[2][0], 0.0, kEps);
    EXPECT_NEAR(k[1][2], 0.0, kEps);
    EXPECT_NEAR(k[2][1], 0.0, kEps);
    EXPECT_NEAR(f[0], 0.0, kEps);
    EXPECT_NEAR(f[1], 20.0, kEps); // never touched (zero-valued bcs)
    EXPECT_NEAR(f[2], 0.0, kEps);
}

// --- gauss_solve ---------------------------------------------------------------

TEST(GaussSolve, SolvesDiagonalSystem) {
    matrix k = {{2.0, 0.0}, {0.0, 3.0}};
    std::vector<double> f = {4.0, 9.0};

    auto u = gauss_solve(k, f);

    ASSERT_EQ(u.size(), 2u);
    EXPECT_NEAR(u[0], 2.0, kEps);
    EXPECT_NEAR(u[1], 3.0, kEps);
}

TEST(GaussSolve, HandlesRequiredPivoting) {
    // zero pivot at [0][0] forces a row swap
    matrix k = {{0.0, 1.0}, {1.0, 1.0}};
    std::vector<double> f = {2.0, 3.0};

    auto u = gauss_solve(k, f);

    EXPECT_NEAR(u[0], 1.0, kEps);
    EXPECT_NEAR(u[1], 2.0, kEps);
}

TEST(GaussSolve, ThrowsOnSingularMatrix) {
    matrix k = {{0.0, 0.0}, {0.0, 0.0}};
    std::vector<double> f = {1.0, 1.0};

    EXPECT_THROW(gauss_solve(k, f), std::runtime_error);
}

// --- reactions -------------------------------------------------------------------

TEST(Reactions, ComputesResidualCorrectly) {
    matrix k = {{4.0, 1.0}, {1.0, 3.0}};
    std::vector<double> u = {1.0, 2.0};
    std::vector<double> f = {5.0, 6.0};
    // K*u = [6, 7]; R = K*u - f = [1, 1]

    auto r = reactions(k, u, f);

    EXPECT_NEAR(r[0], 1.0, kEps);
    EXPECT_NEAR(r[1], 1.0, kEps);
}

// --- elem_stress -----------------------------------------------------------------

TEST(ElemStress, ZeroDisplacementGivesZeroStress) {
    std::vector<node> nodes = {{0.0, 0.0}, {10.0, 0.0}};
    std::vector<elem> elems = {{1, 2, 1.0, 100.0}};
    std::vector<double> u = {0.0, 0.0, 0.0, 0.0};

    auto stresses = elem_stress(nodes, elems, u);

    ASSERT_EQ(stresses.size(), 1u);
    EXPECT_NEAR(stresses[0], 0.0, kEps);
}

TEST(ElemStress, KnownAxialDisplacementOnHorizontalElement) {
    std::vector<node> nodes = {{0.0, 0.0}, {10.0, 0.0}};
    std::vector<elem> elems = {{1, 2, 1.0, 100.0}};
    std::vector<double> u = {0.0, 0.0, 1.0, 0.0}; // node2 moves +1 in x

    auto stresses = elem_stress(nodes, elems, u);
    // strain = du/L = 1/10 = 0.1, stress = E*strain = 100*0.1 = 10
    EXPECT_NEAR(stresses[0], 10.0, kEps);
}

// --- end-to-end regression: global equilibrium ------------------------------------

TEST(SolverIntegration, TriangleGlobalEquilibrium) {
    model m = read_input(std::string(TEST_FIXTURES_DIR) + "/triangle.txt");
    int n_dof = 2 * static_cast<int>(m.nodes.size());

    matrix k_orig = assemble(m.nodes, m.elems);
    std::vector<double> f_full = build_f(m.forces, n_dof);

    matrix k_bc = k_orig;
    std::vector<double> f_bc = f_full;
    apply_bc(k_bc, f_bc, m.bcs);
    std::vector<double> u = gauss_solve(k_bc, f_bc);
    std::vector<double> r = reactions(k_orig, u, f_full);

    // Fixture bcs: node1 dof1(x)=idx0, node1 dof2(y)=idx1, node2 dof2(y)=idx3
    double rx_total = r[0];
    double ry_total = r[1] + r[3];

    double applied_fx = 0.0, applied_fy = 0.0;
    for (const auto &fr : m.forces) {
        if (fr.dof == 1) {
            applied_fx += fr.val;
        } else {
            applied_fy += fr.val;
        }
    }

    // Global force balance: reactions must exactly offset applied loads.
    EXPECT_NEAR(rx_total + applied_fx, 0.0, 1e-6);
    EXPECT_NEAR(ry_total + applied_fy, 0.0, 1e-6);
}
