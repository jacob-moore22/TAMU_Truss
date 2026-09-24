#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "solver.h"
#include "test_helpers.h"

namespace {

void expect_matrix_near(const element_matrix& got, const element_matrix& want,
                        double tol) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(got[i][j], want[i][j], tol)
                << "at (" << i << "," << j << ")";
}

}  // namespace

// ----------------------------------------------------------------
// element_stiffness

TEST(ElementStiffness, HorizontalBar) {
    double k = 200e9 * 1.5 / 10.0;
    element_matrix ke = element_stiffness({0, 0}, {10, 0}, 1.5, 200e9);
    element_matrix want = {
        {{k, 0, -k, 0}, {0, 0, 0, 0}, {-k, 0, k, 0}, {0, 0, 0, 0}}};
    expect_matrix_near(ke, want, k * 1e-12);
}

TEST(ElementStiffness, VerticalBar) {
    double k = 200e9 * 1.5 / 4.0;
    element_matrix ke = element_stiffness({3, 1}, {3, 5}, 1.5, 200e9);
    element_matrix want = {
        {{0, 0, 0, 0}, {0, k, 0, -k}, {0, 0, 0, 0}, {0, -k, 0, k}}};
    expect_matrix_near(ke, want, k * 1e-12);
}

TEST(ElementStiffness, FortyFiveDegreeBar) {
    double l = std::sqrt(2.0);
    double k = 1.0 * 1.0 / l;
    element_matrix ke = element_stiffness({0, 0}, {1, 1}, 1.0, 1.0);
    double h = k / 2.0;
    element_matrix want = {
        {{h, h, -h, -h}, {h, h, -h, -h}, {-h, -h, h, h}, {-h, -h, h, h}}};
    expect_matrix_near(ke, want, 1e-14);
}

TEST(ElementStiffness, IsSymmetric) {
    element_matrix ke = element_stiffness({1, 2}, {4, 7}, 2.0, 3.0);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) EXPECT_DOUBLE_EQ(ke[i][j], ke[j][i]);
}

TEST(ElementStiffness, IndependentOfNodeOrder) {
    element_matrix ab = element_stiffness({1, 2}, {4, 7}, 2.0, 3.0);
    element_matrix ba = element_stiffness({4, 7}, {1, 2}, 2.0, 3.0);
    expect_matrix_near(ab, ba, 1e-14);
}

TEST(ElementStiffness, RowSumsAreZero) {
    // Rigid-body translation (all four dofs equal) produces no force.
    element_matrix ke = element_stiffness({-2, 1}, {3, 4}, 1.7, 2.5e6);
    for (int i = 0; i < 4; ++i) {
        double sum = ke[i][0] + ke[i][1] + ke[i][2] + ke[i][3];
        EXPECT_NEAR(sum, 0.0, 1e-6);
    }
}

// ---------------------------------------------------------------
// assemble_global_stiffness

TEST(AssembleGlobalStiffness, SingleElementPlacedAtCorrectDofs) {
    std::vector<node> nodes = {{0, 0}, {5, 0}, {10, 0}};
    std::vector<element> elems = {{2, 3, 1.0, 1.0}};  // uses nodes 2 and 3
    matrix k = assemble_global_stiffness(nodes, elems);
    ASSERT_EQ(k.size(), 6u);
    element_matrix ke = element_stiffness(nodes[1], nodes[2], 1.0, 1.0);

    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) {
            double want = (i >= 2 && j >= 2) ? ke[i - 2][j - 2] : 0.0;
            EXPECT_DOUBLE_EQ(k[i][j], want) << "at (" << i << "," << j << ")";
        }
}

TEST(AssembleGlobalStiffness, SharedNodeContributionsSum) {
    std::vector<node> nodes = {{0, 0}, {1, 0}, {2, 0}};
    std::vector<element> elems = {{1, 2, 1.0, 1.0}, {2, 3, 1.0, 1.0}};
    matrix k = assemble_global_stiffness(nodes, elems);
    // Each bar has k=1; node 2's x-dof gets 1 from each bar.
    EXPECT_DOUBLE_EQ(k[2][2], 2.0);
    EXPECT_DOUBLE_EQ(k[0][0], 1.0);
    EXPECT_DOUBLE_EQ(k[4][4], 1.0);
    EXPECT_DOUBLE_EQ(k[0][2], -1.0);
    EXPECT_DOUBLE_EQ(k[2][4], -1.0);
    EXPECT_DOUBLE_EQ(k[0][4], 0.0);
}

TEST(AssembleGlobalStiffness, IsSymmetric) {
    model m = triangle_model();
    matrix k = assemble_global_stiffness(m.nodes, m.elements);
    int n = (int)k.size();
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) EXPECT_DOUBLE_EQ(k[i][j], k[j][i]);
}

TEST(AssembleGlobalStiffness, UnreferencedNodeRowsStayZero) {
    std::vector<node> nodes = {{0, 0}, {1, 0}, {5, 5}};
    std::vector<element> elems = {{1, 2, 1.0, 1.0}};
    matrix k = assemble_global_stiffness(nodes, elems);
    for (int j = 0; j < 6; ++j) {
        EXPECT_DOUBLE_EQ(k[4][j], 0.0);
        EXPECT_DOUBLE_EQ(k[5][j], 0.0);
    }
}

// ----------------------------------------------------------------
// build_load_vector

TEST(BuildLoadVector, SizeAndDefaultZero) {
    std::vector<double> f = build_load_vector({}, 8);
    ASSERT_EQ(f.size(), 8u);
    for (double v : f) EXPECT_DOUBLE_EQ(v, 0.0);
}

TEST(BuildLoadVector, MapsNodeDofToIndex) {
    std::vector<double> f =
        build_load_vector({{3, 2, -10000.0}, {1, 1, 42.0}}, 6);
    EXPECT_DOUBLE_EQ(f[5], -10000.0);
    EXPECT_DOUBLE_EQ(f[0], 42.0);
    EXPECT_DOUBLE_EQ(f[1], 0.0);
    EXPECT_DOUBLE_EQ(f[4], 0.0);
}

TEST(BuildLoadVector, SameDofAccumulates) {
    std::vector<double> f = build_load_vector({{2, 1, 3.0}, {2, 1, 4.0}}, 4);
    EXPECT_DOUBLE_EQ(f[2], 7.0);
}

// ---------------------------------------------------------------
// apply_boundary_conditions

TEST(ApplyBoundaryConditions, ZeroBcClearsRowAndColumn) {
    matrix k = {{4, 1, 2}, {1, 5, 3}, {2, 3, 6}};
    std::vector<double> f = {10, 20, 30};
    apply_boundary_conditions(k, f, {{1, 2, 0.0}});  // dof index 1
    EXPECT_DOUBLE_EQ(k[1][1], 1.0);
    EXPECT_DOUBLE_EQ(k[1][0], 0.0);
    EXPECT_DOUBLE_EQ(k[1][2], 0.0);
    EXPECT_DOUBLE_EQ(k[0][1], 0.0);
    EXPECT_DOUBLE_EQ(k[2][1], 0.0);
    EXPECT_DOUBLE_EQ(f[1], 0.0);
    // untouched entries
    EXPECT_DOUBLE_EQ(k[0][0], 4.0);
    EXPECT_DOUBLE_EQ(k[0][2], 2.0);
    EXPECT_DOUBLE_EQ(k[2][2], 6.0);
    EXPECT_DOUBLE_EQ(f[0], 10.0);
    EXPECT_DOUBLE_EQ(f[2], 30.0);
}

TEST(ApplyBoundaryConditions, PrescribedDisplacementShiftsLoad) {
    matrix k = {{4, 1, 2}, {1, 5, 3}, {2, 3, 6}};
    std::vector<double> f = {10, 20, 30};
    double val = 0.5;
    apply_boundary_conditions(k, f, {{1, 2, val}});  // dof index 1
    EXPECT_DOUBLE_EQ(f[1], val);
    EXPECT_DOUBLE_EQ(f[0], 10.0 - 1.0 * val);
    EXPECT_DOUBLE_EQ(f[2], 30.0 - 3.0 * val);
    EXPECT_DOUBLE_EQ(k[1][1], 1.0);
}

TEST(ApplyBoundaryConditions, MultipleBcs) {
    model m = triangle_model();
    matrix k = assemble_global_stiffness(m.nodes, m.elements);
    std::vector<double> f = build_load_vector(m.loads, 6);
    apply_boundary_conditions(k, f, m.supports);
    for (int d : {0, 1, 3}) {
        EXPECT_DOUBLE_EQ(k[d][d], 1.0);
        EXPECT_DOUBLE_EQ(f[d], 0.0);
        for (int j = 0; j < 6; ++j) {
            if (j != d) {
                EXPECT_DOUBLE_EQ(k[d][j], 0.0);
                EXPECT_DOUBLE_EQ(k[j][d], 0.0);
            }
        }
    }
}

// ------------------------------------------------------------ gauss_solve

TEST(GaussSolve, IdentityReturnsRhs) {
    matrix k = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    std::vector<double> u = gauss_solve(k, {3, -2, 7});
    EXPECT_DOUBLE_EQ(u[0], 3.0);
    EXPECT_DOUBLE_EQ(u[1], -2.0);
    EXPECT_DOUBLE_EQ(u[2], 7.0);
}

TEST(GaussSolve, KnownSystem) {
    // x = [1, 2, 3]
    matrix k = {{2, 1, -1}, {-3, -1, 2}, {-2, 1, 2}};
    std::vector<double> f = {2 + 2 - 3, -3 - 2 + 6, -2 + 2 + 6};
    std::vector<double> u = gauss_solve(k, f);
    EXPECT_NEAR(u[0], 1.0, 1e-12);
    EXPECT_NEAR(u[1], 2.0, 1e-12);
    EXPECT_NEAR(u[2], 3.0, 1e-12);
}

TEST(GaussSolve, RequiresPivoting) {
    // Zero on the leading diagonal but nonsingular; x = [1, 1]
    matrix k = {{0, 1}, {1, 0}};
    std::vector<double> u = gauss_solve(k, {2, 3});
    EXPECT_NEAR(u[0], 3.0, 1e-12);
    EXPECT_NEAR(u[1], 2.0, 1e-12);
}

TEST(GaussSolve, SingularThrows) {
    matrix k = {{1, 2}, {2, 4}};
    EXPECT_THROW(gauss_solve(k, {1, 2}), std::runtime_error);
}

TEST(GaussSolve, DoesNotModifyInputs) {
    matrix k = {{2, 1}, {1, 3}};
    std::vector<double> f = {1, 2};
    std::vector<double> u1 = gauss_solve(k, f);
    EXPECT_DOUBLE_EQ(k[0][0], 2.0);
    EXPECT_DOUBLE_EQ(k[1][0], 1.0);
    EXPECT_DOUBLE_EQ(f[1], 2.0);
    std::vector<double> u2 = gauss_solve(k, f);
    EXPECT_DOUBLE_EQ(u1[0], u2[0]);
    EXPECT_DOUBLE_EQ(u1[1], u2[1]);
}

// -------------------------------------------------------------- reactions

TEST(ComputeReactions, ComputesKuMinusF) {
    matrix k = {{2, 1}, {1, 3}};
    std::vector<double> u = {1, 2};
    std::vector<double> f = {0.5, 1.0};
    std::vector<double> r = compute_reactions(k, u, f);
    EXPECT_DOUBLE_EQ(r[0], 2 * 1 + 1 * 2 - 0.5);
    EXPECT_DOUBLE_EQ(r[1], 1 * 1 + 3 * 2 - 1.0);
}

// ------------------------------------------------------------
// compute_element_stresses

TEST(ComputeElementStresses, AxialStretchOfHorizontalBar) {
    std::vector<node> nodes = {{0, 0}, {2, 0}};
    std::vector<element> elems = {{1, 2, 1.0, 100.0}};
    std::vector<double> u = {0, 0, 0.01, 0};
    std::vector<double> s = compute_element_stresses(nodes, elems, u);
    ASSERT_EQ(s.size(), 1u);
    EXPECT_NEAR(s[0], 100.0 * 0.01 / 2.0, 1e-12);
}

TEST(ComputeElementStresses, RigidBodyTranslationIsStressFree) {
    model m = triangle_model();
    std::vector<double> u = {1, -2, 1, -2, 1, -2};
    std::vector<double> s = compute_element_stresses(m.nodes, m.elements, u);
    for (double v : s) EXPECT_NEAR(v, 0.0, 1e-3);  // E=200e9 amplifies noise
}

TEST(ComputeElementStresses, CompressionIsNegative) {
    std::vector<node> nodes = {{0, 0}, {0, 3}};
    std::vector<element> elems = {{1, 2, 1.0, 10.0}};
    std::vector<double> u = {0, 0, 0, -0.3};  // node 2 moves toward node 1
    std::vector<double> s = compute_element_stresses(nodes, elems, u);
    EXPECT_NEAR(s[0], -10.0 * 0.3 / 3.0, 1e-12);
}

// ------------------------------------------------------- full pipeline

TEST(FullSolve, TriangleMatchesReferenceSolution) {
    model m = triangle_model();
    int n_dof = 6;
    matrix k_orig = assemble_global_stiffness(m.nodes, m.elements);
    std::vector<double> f = build_load_vector(m.loads, n_dof);
    matrix k = k_orig;
    std::vector<double> f_bc = f;
    apply_boundary_conditions(k, f_bc, m.supports);
    std::vector<double> u = gauss_solve(k, f_bc);
    std::vector<double> r = compute_reactions(k_orig, u, f);
    std::vector<double> s = compute_element_stresses(m.nodes, m.elements, u);

    for (int i = 0; i < 6; ++i)
        EXPECT_NEAR(u[i], kTriRefU[i], 1e-9 * 5.5e-7) << "u[" << i << "]";

    EXPECT_NEAR(r[0], -5000.0, 1e-6);
    EXPECT_NEAR(r[1], 0.0, 1e-6);
    EXPECT_NEAR(r[2], 0.0, 1e-6);
    EXPECT_NEAR(r[3], 10000.0, 1e-6);
    EXPECT_NEAR(r[4], 0.0, 1e-6);
    EXPECT_NEAR(r[5], 0.0, 1e-6);

    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(s[i], kTriRefStress[i], 1e-9 * 7500.0) << "s[" << i << "]";
}

// Characterization test: documents CURRENT behaviour, not a spec.
// TODO(solver): the singular-pivot check in gauss_solve is an absolute
// |pivot| < 1e-12. With realistic stiffness (E*A/L ~ 1e10) the round-off left
// on the pivot of a singular matrix is orders of magnitude above that, so an
// unconstrained model is NOT detected and garbage displacements are returned.
// A relative check (pivot / max |K|) would catch it.
TEST(GaussSolve, SingularCheckIsAbsolute) {
    model m = triangle_model();  // E = 200e9
    m.supports.clear();
    matrix k = assemble_global_stiffness(m.nodes, m.elements);
    std::vector<double> f = build_load_vector(m.loads, 6);
    EXPECT_NO_THROW(gauss_solve(k, f));

    for (auto& e : m.elements)
        e.youngs_modulus = 1.0;  // O(1) entries: check fires
    matrix k1 = assemble_global_stiffness(m.nodes, m.elements);
    EXPECT_THROW(gauss_solve(k1, f), std::runtime_error);
}
