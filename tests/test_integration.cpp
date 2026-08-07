// Full-pipeline regression test: hand-built model equivalent to
// examples/input_triangle.txt (read_input is intentionally never called —
// io.cpp is out of scope for unit testing), run through
// assemble -> apply_bc -> gauss_solve -> reactions -> elem_stress at full
// load (factor = 1.0, i.e. the last load step main() would run).
//
// Expected values were computed independently in Python/numpy (direct
// stiffness assembly + np.linalg.solve), not by re-running this codebase,
// so this test can actually catch a regression in the solver pipeline.

#include <gtest/gtest.h>

#include <cmath>

#include "solver.h"
#include "types.h"

namespace {

// Relative tolerance for large-magnitude engineering values (forces,
// reactions, stresses), plus a small absolute floor so expected-zero values
// don't demand an unreasonable number of digits.
void ExpectClose(double actual, double expected, double abs_floor = 1e-2, double rel_tol = 1e-4) {
    double tol = std::max(abs_floor, std::fabs(expected) * rel_tol);
    EXPECT_NEAR(actual, expected, tol) << "expected " << expected << ", got " << actual;
}

model BuildTriangleModel() {
    model m;
    m.nodes = {
        {0.0, 0.0},
        {10.0, 0.0},
        {5.0, 10.0},
    };
    m.elements = {
        {1, 2, 1.5, 200e9},
        {2, 3, 1.5, 200e9},
        {1, 3, 1.5, 200e9},
    };
    m.boundary_conditions = {
        {1, 1, 0.0},
        {1, 2, 0.0},
        {2, 2, 0.0},
    };
    m.forces = {
        {3, 1, 5000.0},
        {3, 2, -10000.0},
    };
    m.load_steps = 1;
    return m;
}

}  // namespace

TEST(TriangleTrussIntegration, FullPipelineMatchesIndependentReferenceSolution) {
    model m = BuildTriangleModel();
    int dof_count = 2 * (int)m.nodes.size();

    matrix global_stiffness = assemble(m.nodes, m.elements);
    std::vector<double> applied_force = build_f(m.forces, dof_count);

    matrix stiffness_with_bcs = global_stiffness;
    std::vector<double> force_with_bcs = applied_force;
    apply_bc(stiffness_with_bcs, force_with_bcs, m.boundary_conditions);

    std::vector<double> displacements = gauss_solve(stiffness_with_bcs, force_with_bcs);
    std::vector<double> reaction_forces = reactions(global_stiffness, displacements, applied_force);
    std::vector<double> stresses = elem_stress(m.nodes, m.elements, displacements);

    ASSERT_EQ(displacements.size(), 6u);
    // Reference (numpy): [0, 0, 1.66666667e-07, 0, 5.49180829e-07, -2.74590414e-07]
    ExpectClose(displacements[0], 0.0, 1e-12);
    ExpectClose(displacements[1], 0.0, 1e-12);
    ExpectClose(displacements[2], 1.66666667e-07, 1e-12);
    ExpectClose(displacements[3], 0.0, 1e-12);
    ExpectClose(displacements[4], 5.49180829e-07, 1e-12);
    ExpectClose(displacements[5], -2.74590414e-07, 1e-12);

    ASSERT_EQ(reaction_forces.size(), 6u);
    // Reference (numpy): [-5000, ~0, ~0, 10000, ~0, ~0]
    ExpectClose(reaction_forces[0], -5000.0);
    ExpectClose(reaction_forces[1], 0.0);
    ExpectClose(reaction_forces[2], 0.0);
    ExpectClose(reaction_forces[3], 10000.0);
    ExpectClose(reaction_forces[4], 0.0);
    ExpectClose(reaction_forces[5], 0.0);

    // Global equilibrium: reactions + applied forces sum to zero.
    double sum_x = reaction_forces[0] + reaction_forces[2] + reaction_forces[4] + 5000.0;
    double sum_y = reaction_forces[1] + reaction_forces[3] + reaction_forces[5] - 10000.0;
    EXPECT_NEAR(sum_x, 0.0, 1e-2);
    EXPECT_NEAR(sum_y, 0.0, 1e-2);

    ASSERT_EQ(stresses.size(), 3u);
    // Reference (numpy): elem(1-2)=3333.333..., elem(2-3)=-7453.560, elem(1-3)=0.0
    ExpectClose(stresses[0], 3333.3333333333326);
    ExpectClose(stresses[1], -7453.5599249993);
    ExpectClose(stresses[2], 0.0);
}
