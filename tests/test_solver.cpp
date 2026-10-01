/**
 * @file test_solver.cpp
 * @brief Tests for the direct stiffness method routines in solver.h.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "io.h"
#include "solver.h"
#include "test_helpers.h"

namespace {

/// Young's modulus used by the synthetic test bars (steel, Pa).
constexpr double kYoungsModulus = 200e9;
/// Cross-sectional area used by the synthetic test bars.
constexpr double kArea = 1.5;

/**
 * @brief Absolute tolerance scaled to the magnitude being compared.
 * @param magnitude Typical size of the compared values.
 * @return 1e-9 times max(1, |magnitude|).
 */
double tol(double magnitude) {
    return 1e-9 * std::max(1.0, std::fabs(magnitude));
}

}  // namespace

// ---- k_local ----

/// @brief A bar along x only has stiffness in the x DOFs.
TEST(KLocal, HorizontalBarStiffnessOnlyInX) {
    auto element_stiffness = k_local({0, 0}, {2, 0}, kArea, kYoungsModulus);
    double expected_stiffness = kYoungsModulus * kArea / 2.0;
    EXPECT_NEAR(element_stiffness[0][0], expected_stiffness,
                tol(expected_stiffness));
    EXPECT_NEAR(element_stiffness[0][2], -expected_stiffness,
                tol(expected_stiffness));
    EXPECT_NEAR(element_stiffness[2][2], expected_stiffness,
                tol(expected_stiffness));
    EXPECT_NEAR(element_stiffness[1][1], 0.0, tol(expected_stiffness));
    EXPECT_NEAR(element_stiffness[3][3], 0.0, tol(expected_stiffness));
    EXPECT_NEAR(element_stiffness[0][1], 0.0, tol(expected_stiffness));
}

/// @brief A bar along y only has stiffness in the y DOFs.
TEST(KLocal, VerticalBarStiffnessOnlyInY) {
    auto element_stiffness = k_local({0, 0}, {0, 4}, kArea, kYoungsModulus);
    double expected_stiffness = kYoungsModulus * kArea / 4.0;
    EXPECT_NEAR(element_stiffness[1][1], expected_stiffness,
                tol(expected_stiffness));
    EXPECT_NEAR(element_stiffness[1][3], -expected_stiffness,
                tol(expected_stiffness));
    EXPECT_NEAR(element_stiffness[3][3], expected_stiffness,
                tol(expected_stiffness));
    EXPECT_NEAR(element_stiffness[0][0], 0.0, tol(expected_stiffness));
    EXPECT_NEAR(element_stiffness[2][2], 0.0, tol(expected_stiffness));
}

/// @brief A 45-degree bar splits EA/L equally between all terms.
TEST(KLocal, DiagonalBarSplitsStiffnessEvenly) {
    auto element_stiffness = k_local({0, 0}, {3, 3}, kArea, kYoungsModulus);
    double length = std::sqrt(18.0);
    double half_stiffness = kYoungsModulus * kArea / length / 2.0;
    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 2; ++col) {
            EXPECT_NEAR(element_stiffness[row][col], half_stiffness,
                        tol(half_stiffness));
            EXPECT_NEAR(element_stiffness[row][col + 2], -half_stiffness,
                        tol(half_stiffness));
        }
    }
}

/// @brief The element matrix is symmetric and rows sum to zero (rigid-body
///        translation produces no force).
TEST(KLocal, SymmetricWithZeroRowSums) {
    auto element_stiffness = k_local({1, 2}, {4, -2}, kArea, kYoungsModulus);
    for (int row = 0; row < 4; ++row) {
        double row_sum = 0.0;
        for (int col = 0; col < 4; ++col) {
            EXPECT_NEAR(element_stiffness[row][col],
                        element_stiffness[col][row],
                        tol(element_stiffness[row][col]));
            row_sum += element_stiffness[row][col];
        }
        EXPECT_NEAR(row_sum, 0.0, tol(element_stiffness[row][row]));
    }
}

// ---- assemble ----

/// @brief The global matrix is (2N x 2N).
TEST(Assemble, SizeIsTwiceNodeCount) {
    std::vector<node> nodes = {{0, 0}, {1, 0}, {0, 1}};
    matrix stiffness = assemble(nodes, {{1, 2, kArea, kYoungsModulus}});
    ASSERT_EQ(stiffness.size(), 6u);
    for (const auto& matrix_row : stiffness) EXPECT_EQ(matrix_row.size(), 6u);
}

/// @brief With one element the global matrix equals k_local().
TEST(Assemble, SingleElementMatchesKLocal) {
    std::vector<node> nodes = {{0, 0}, {3, 4}};
    matrix stiffness = assemble(nodes, {{1, 2, kArea, kYoungsModulus}});
    auto element_stiffness = k_local(nodes[0], nodes[1], kArea, kYoungsModulus);
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            EXPECT_DOUBLE_EQ(stiffness[row][col], element_stiffness[row][col]);
}

/// @brief Contributions add at a shared node; the result is symmetric.
TEST(Assemble, SharedNodeAccumulatesAndIsSymmetric) {
    // Two collinear bars meeting at node 2.
    std::vector<node> nodes = {{0, 0}, {1, 0}, {3, 0}};
    std::vector<elem> elements = {{1, 2, kArea, kYoungsModulus},
                                  {2, 3, kArea, kYoungsModulus}};
    matrix stiffness = assemble(nodes, elements);
    double short_bar_stiffness = kYoungsModulus * kArea / 1.0;
    double long_bar_stiffness = kYoungsModulus * kArea / 2.0;
    EXPECT_NEAR(stiffness[2][2], short_bar_stiffness + long_bar_stiffness,
                tol(short_bar_stiffness));
    // Nodes 1 and 3 are not connected.
    EXPECT_NEAR(stiffness[0][4], 0.0, tol(short_bar_stiffness));
    for (size_t row = 0; row < stiffness.size(); ++row)
        for (size_t col = 0; col < stiffness.size(); ++col)
            EXPECT_DOUBLE_EQ(stiffness[row][col], stiffness[col][row]);
}

// ---- build_f ----

/// @brief Loads land at the 0-based global DOF; other entries stay zero.
TEST(BuildF, PlacesForcesAtGlobalDofs) {
    auto loads = build_f({{2, 1, 7.0}, {3, 2, -4.0}}, 6);
    ASSERT_EQ(loads.size(), 6u);
    EXPECT_DOUBLE_EQ(loads[2], 7.0);
    EXPECT_DOUBLE_EQ(loads[5], -4.0);
    EXPECT_DOUBLE_EQ(loads[0], 0.0);
    EXPECT_DOUBLE_EQ(loads[1], 0.0);
    EXPECT_DOUBLE_EQ(loads[3], 0.0);
    EXPECT_DOUBLE_EQ(loads[4], 0.0);
}

/// @brief Two loads on the same DOF are summed.
TEST(BuildF, RepeatedForcesOnSameDofAdd) {
    auto loads = build_f({{1, 2, 3.0}, {1, 2, 4.5}}, 2);
    EXPECT_DOUBLE_EQ(loads[1], 7.5);
}

// ---- apply_bc ----

/// @brief The constrained row/column are zeroed with 1 on the diagonal.
TEST(ApplyBc, ZeroesRowAndColumnWithUnitDiagonal) {
    matrix stiffness = {
        {4, 1, 2, 3}, {1, 5, 6, 7}, {2, 6, 8, 9}, {3, 7, 9, 10}};
    std::vector<double> loads = {1, 2, 3, 4};
    apply_bc(stiffness, loads, {{1, 2, 0.0}});  // global dof 1
    for (int index = 0; index < 4; ++index) {
        if (index == 1) continue;
        EXPECT_DOUBLE_EQ(stiffness[1][index], 0.0);
        EXPECT_DOUBLE_EQ(stiffness[index][1], 0.0);
    }
    EXPECT_DOUBLE_EQ(stiffness[1][1], 1.0);
    EXPECT_DOUBLE_EQ(stiffness[0][0], 4.0);  // untouched
    EXPECT_DOUBLE_EQ(loads[1], 0.0);
    EXPECT_DOUBLE_EQ(loads[0], 1.0);  // zero prescribed value: rhs unchanged
}

/// @brief A non-zero prescribed displacement is moved to the right-hand side
///        and recovered by the solve.
TEST(ApplyBc, NonZeroPrescribedDisplacementMovesToRhs) {
    matrix stiffness = {{2, -1}, {-1, 2}};
    std::vector<double> loads = {0, 0};
    apply_bc(stiffness, loads, {{1, 1, 0.5}});
    EXPECT_DOUBLE_EQ(loads[0], 0.5);
    EXPECT_DOUBLE_EQ(loads[1], 0.5);  // 0 - (-1)*0.5
    auto displacements = gauss_solve(stiffness, loads);
    EXPECT_NEAR(displacements[0], 0.5, 1e-12);
    EXPECT_NEAR(displacements[1], 0.25, 1e-12);
}

// ---- gauss_solve ----

/// @brief Solves a 2x2 system.
TEST(GaussSolve, Solves2x2) {
    auto solution = gauss_solve({{2, 1}, {1, 3}}, {3, 5});
    EXPECT_NEAR(solution[0], 0.8, 1e-12);
    EXPECT_NEAR(solution[1], 1.4, 1e-12);
}

/// @brief Solves a 3x3 system.
TEST(GaussSolve, Solves3x3) {
    auto solution =
        gauss_solve({{4, -2, 1}, {-2, 4, -2}, {1, -2, 4}}, {11, -16, 17});
    EXPECT_NEAR(solution[0], 1.0, 1e-12);
    EXPECT_NEAR(solution[1], -2.0, 1e-12);
    EXPECT_NEAR(solution[2], 3.0, 1e-12);
}

/// @brief Row swapping handles a zero on the initial diagonal.
TEST(GaussSolve, PivotsAroundZeroDiagonal) {
    auto solution = gauss_solve({{0, 1}, {1, 0}}, {2, 3});
    EXPECT_NEAR(solution[0], 3.0, 1e-12);
    EXPECT_NEAR(solution[1], 2.0, 1e-12);
}

/// @brief A singular matrix throws.
TEST(GaussSolve, ThrowsOnSingularMatrix) {
    EXPECT_THROW(gauss_solve({{1, 2}, {2, 4}}, {1, 2}), std::runtime_error);
}

/// @brief An unsupported truss (rigid-body modes) throws.
TEST(GaussSolve, ThrowsOnUnconstrainedTruss) {
    std::vector<node> nodes = {{0, 0}, {1, 0}};
    matrix stiffness = assemble(nodes, {{1, 2, kArea, kYoungsModulus}});
    EXPECT_THROW(gauss_solve(stiffness, std::vector<double>(4, 0.0)),
                 std::runtime_error);
}

/// @brief The caller's matrix and vector are not modified.
TEST(GaussSolve, DoesNotModifyCallerInputs) {
    matrix coefficients = {{0, 1}, {1, 0}};
    std::vector<double> rhs = {2, 3};
    gauss_solve(coefficients, rhs);
    EXPECT_EQ(coefficients, (matrix{{0, 1}, {1, 0}}));
    EXPECT_EQ(rhs, (std::vector<double>{2, 3}));
}

// ---- reactions ----

/// @brief The support reaction of a single loaded bar balances the load.
TEST(Reactions, SingleBarReactionBalancesLoad) {
    // Bar along x, node 1 pinned, node 2 on a y-roller, pulled with P in x.
    std::vector<node> nodes = {{0, 0}, {2, 0}};
    std::vector<elem> elements = {{1, 2, kArea, kYoungsModulus}};
    std::vector<bc> boundary_conditions = {{1, 1, 0}, {1, 2, 0}, {2, 2, 0}};
    double applied_load = 1000.0;
    matrix stiffness = assemble(nodes, elements);
    auto loads = build_f({{2, 1, applied_load}}, 4);
    matrix constrained_stiffness = stiffness;
    auto constrained_loads = loads;
    apply_bc(constrained_stiffness, constrained_loads, boundary_conditions);
    auto displacements = gauss_solve(constrained_stiffness, constrained_loads);
    EXPECT_NEAR(displacements[2], applied_load * 2.0 / (kYoungsModulus * kArea),
                1e-15);

    auto reaction_forces = reactions(stiffness, displacements, loads);
    EXPECT_NEAR(reaction_forces[0], -applied_load, tol(applied_load));
    EXPECT_NEAR(reaction_forces[2], 0.0, tol(applied_load));  // free dof
    EXPECT_NEAR(reaction_forces[1], 0.0, tol(applied_load));
    EXPECT_NEAR(reaction_forces[3], 0.0, tol(applied_load));
}

// ---- elem_stress ----

/// @brief Stretching a bar by delta gives stress E * delta / L.
TEST(ElemStress, AxialStretchGivesEDeltaOverL) {
    std::vector<node> nodes = {{0, 0}, {4, 0}};
    double stretch = 1e-3;
    auto axial_stresses =
        elem_stress(nodes, {{1, 2, kArea, kYoungsModulus}}, {0, 0, stretch, 0});
    ASSERT_EQ(axial_stresses.size(), 1u);
    EXPECT_NEAR(axial_stresses[0], kYoungsModulus * stretch / 4.0,
                tol(kYoungsModulus * stretch));
}

/// @brief Shortening a bar gives negative (compressive) stress.
TEST(ElemStress, CompressionIsNegative) {
    std::vector<node> nodes = {{0, 0}, {3, 4}};
    // Move node 2 toward node 1 along the bar axis by 0.01.
    std::vector<double> displacements = {0, 0, -0.006, -0.008};
    auto axial_stresses =
        elem_stress(nodes, {{1, 2, kArea, kYoungsModulus}}, displacements);
    EXPECT_NEAR(axial_stresses[0], -kYoungsModulus * 0.01 / 5.0,
                tol(kYoungsModulus * 0.01));
}

/// @brief Rigid translation and infinitesimal rotation give zero stress.
TEST(ElemStress, RigidBodyMotionGivesZeroStress) {
    std::vector<node> nodes = {{1, 2}, {4, 6}};
    std::vector<elem> elements = {{1, 2, kArea, kYoungsModulus}};
    auto translation_stresses =
        elem_stress(nodes, elements, {0.3, -0.2, 0.3, -0.2});
    EXPECT_NEAR(translation_stresses[0], 0.0, 1e-3);
    // Infinitesimal rotation u = theta * (-y, x).
    double rotation_angle = 1e-4;
    auto rotation_stresses =
        elem_stress(nodes, elements,
                    {-rotation_angle * 2, rotation_angle * 1,
                     -rotation_angle * 6, rotation_angle * 4});
    EXPECT_NEAR(rotation_stresses[0], 0.0, 1e-3);
}

// ---- integration: triangle example vs. method of joints ----

/// @brief The full pipeline on the triangle example matches a hand
///        calculation by the method of joints.
TEST(SolverIntegration, TriangleMatchesHandCalculation) {
    model truss_model = read_input(example_path("input_triangle.txt"));
    int num_dofs = 2 * (int)truss_model.nodes.size();
    matrix stiffness = assemble(truss_model.nodes, truss_model.elements);
    auto loads = build_f(truss_model.forces, num_dofs);
    matrix constrained_stiffness = stiffness;
    auto constrained_loads = loads;
    apply_bc(constrained_stiffness, constrained_loads,
             truss_model.boundary_conditions);
    auto displacements = gauss_solve(constrained_stiffness, constrained_loads);
    auto reaction_forces = reactions(stiffness, displacements, loads);
    auto axial_stresses =
        elem_stress(truss_model.nodes, truss_model.elements, displacements);

    // Supports: node 1 pinned, node 2 roller in y. Load (5000, -10000) at 3.
    EXPECT_NEAR(reaction_forces[0], -5000.0, 1e-3);
    EXPECT_NEAR(reaction_forces[1], 0.0, 1e-3);
    EXPECT_NEAR(reaction_forces[3], 10000.0, 1e-3);
    EXPECT_NEAR(reaction_forces[0] + 5000.0, 0.0, 1e-3);
    EXPECT_NEAR(reaction_forces[1] + reaction_forces[3] - 10000.0, 0.0, 1e-3);

    // Member forces: 1-2 = +5000, 2-3 = -1000*sqrt(125), 1-3 = 0.
    EXPECT_NEAR(axial_stresses[0], 5000.0 / 1.5, 1e-3);
    EXPECT_NEAR(axial_stresses[1], -1000.0 * std::sqrt(125.0) / 1.5, 1e-3);
    EXPECT_NEAR(axial_stresses[2], 0.0, 1e-3);
}
