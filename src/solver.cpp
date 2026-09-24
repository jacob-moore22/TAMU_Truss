/**
 * @file solver.cpp
 * @brief Implementation of the direct stiffness method (see solver.h).
 */
#include "solver.h"

#include <cmath>
#include <stdexcept>

element_matrix element_stiffness(const node& node_1, const node& node_2,
                                 double area, double youngs_modulus) {
    // Geometry of the bar: its length and orientation angle theta measured
    // from the global X axis. Only cos(theta) and sin(theta) are needed.
    double delta_x = node_2.x - node_1.x;  // horizontal run
    double delta_y = node_2.y - node_1.y;  // vertical rise
    double length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    double cos_theta = delta_x / length;  // often written "c"
    double sin_theta = delta_y / length;  // often written "s"

    // Axial stiffness of the bar, k = EA/L (force per unit stretch).
    double axial_stiffness = youngs_modulus * area / length;

    // Shorthand so the matrix below reads like the textbook formula.
    double k = axial_stiffness;
    double c = cos_theta;
    double s = sin_theta;

    element_matrix k_element = {
        {{k * c * c, k * c * s, -k * c * c, -k * c * s},
         {k * c * s, k * s * s, -k * c * s, -k * s * s},
         {-k * c * c, -k * c * s, k * c * c, k * c * s},
         {-k * c * s, -k * s * s, k * c * s, k * s * s}}};
    return k_element;
}

matrix assemble_global_stiffness(const std::vector<node>& nodes,
                                 const std::vector<element>& elements) {
    // Two DOFs (X and Y) per node.
    int num_dofs = 2 * (int)nodes.size();

    // Global stiffness matrix, initially all zeros.
    matrix k_global(num_dofs, std::vector<double>(num_dofs, 0.0));

    for (const auto& el : elements) {
        element_matrix k_element =
            element_stiffness(nodes[el.node_1 - 1], nodes[el.node_2 - 1],
                              el.area, el.youngs_modulus);

        // Global DOF index for each of the element's four local DOFs, in the
        // order [u1x, u1y, u2x, u2y] matching the rows of k_element.
        int global_dofs[4] = {2 * (el.node_1 - 1), 2 * (el.node_1 - 1) + 1,
                              2 * (el.node_2 - 1), 2 * (el.node_2 - 1) + 1};

        // Scatter-add: each entry of the 4x4 element matrix is added to the
        // global matrix at the corresponding pair of global DOFs.
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                k_global[global_dofs[i]][global_dofs[j]] += k_element[i][j];
            }
        }
    }
    return k_global;
}

std::vector<double> build_load_vector(const std::vector<force>& loads,
                                      int num_dofs) {
    // Global load vector {F}, one entry per DOF, initially all zeros.
    std::vector<double> load_vector(num_dofs, 0.0);
    for (const auto& load : loads) {
        // Convert (node, direction) to the single global DOF index.
        int global_dof = 2 * (load.node_id - 1) + (load.dof - 1);
        load_vector[global_dof] += load.value;
    }
    return load_vector;
}

void apply_boundary_conditions(
    matrix& k_global, std::vector<double>& load_vector,
    const std::vector<boundary_condition>& supports) {
    int num_dofs = (int)load_vector.size();
    for (const auto& support : supports) {
        // Global DOF index being constrained.
        int constrained_dof = 2 * (support.node_id - 1) + (support.dof - 1);

        // A prescribed (possibly non-zero) displacement contributes a known
        // force  K[i][dof] * value  to every other equation; move it to the
        // right-hand side.
        for (int i = 0; i < num_dofs; ++i) {
            load_vector[i] -= k_global[i][constrained_dof] * support.value;
        }

        // Replace the constrained equation with  1 * u[dof] = value  by
        // zeroing its row and column and putting 1 on the diagonal.
        for (int i = 0; i < num_dofs; ++i) {
            k_global[constrained_dof][i] = 0.0;
            k_global[i][constrained_dof] = 0.0;
        }
        k_global[constrained_dof][constrained_dof] = 1.0;
        load_vector[constrained_dof] = support.value;
    }
}

std::vector<double> gauss_solve(matrix k_global,
                                std::vector<double> load_vector) {
    // Number of equations (= number of DOFs). The matrix and vector were
    // passed by value, so they can be modified freely here.
    int num_equations = (int)load_vector.size();

    // ---- Forward elimination: reduce [K] to upper-triangular form. ----
    for (int pivot_row = 0; pivot_row < num_equations; ++pivot_row) {
        // Partial pivoting: find the row at or below pivot_row with the
        // largest absolute value in this column, and swap it into place.
        // This avoids dividing by a tiny number and improves accuracy.
        int max_row = pivot_row;
        double max_abs_value = std::fabs(k_global[pivot_row][pivot_row]);
        for (int i = pivot_row + 1; i < num_equations; ++i) {
            if (std::fabs(k_global[i][pivot_row]) > max_abs_value) {
                max_abs_value = std::fabs(k_global[i][pivot_row]);
                max_row = i;
            }
        }
        if (max_row != pivot_row) {
            std::swap(k_global[pivot_row], k_global[max_row]);
            std::swap(load_vector[pivot_row], load_vector[max_row]);
        }

        // A zero pivot means the matrix is singular: the truss is a
        // mechanism or has too few supports.
        if (std::fabs(k_global[pivot_row][pivot_row]) < 1e-12) {
            throw std::runtime_error(
                "singular stiffness matrix - check boundary conditions");
        }

        // Eliminate the pivot column from every row below the pivot row.
        for (int i = pivot_row + 1; i < num_equations; ++i) {
            double elimination_factor =
                k_global[i][pivot_row] / k_global[pivot_row][pivot_row];
            for (int j = pivot_row; j < num_equations; ++j) {
                k_global[i][j] -= elimination_factor * k_global[pivot_row][j];
            }
            load_vector[i] -= elimination_factor * load_vector[pivot_row];
        }
    }

    // ---- Back substitution: solve from the last equation upward. ----
    std::vector<double> displacements(num_equations, 0.0);
    for (int i = num_equations - 1; i >= 0; --i) {
        // Start from the right-hand side and subtract the already-known
        // unknowns to the right of the diagonal.
        double remaining = load_vector[i];
        for (int j = i + 1; j < num_equations; ++j) {
            remaining -= k_global[i][j] * displacements[j];
        }
        displacements[i] = remaining / k_global[i][i];
    }
    return displacements;
}

std::vector<double> compute_reactions(const matrix& k_global,
                                      const std::vector<double>& displacements,
                                      const std::vector<double>& load_vector) {
    int num_dofs = (int)displacements.size();

    // Reaction at each DOF: {R} = [K]{u} - {F}. At a free DOF the internal
    // forces balance the applied load, so the reaction is ~0.
    std::vector<double> reactions(num_dofs, 0.0);
    for (int i = 0; i < num_dofs; ++i) {
        double internal_force = 0.0;  // i-th entry of [K]{u}
        for (int j = 0; j < num_dofs; ++j) {
            internal_force += k_global[i][j] * displacements[j];
        }
        reactions[i] = internal_force - load_vector[i];
    }
    return reactions;
}

std::vector<double> compute_element_stresses(
    const std::vector<node>& nodes, const std::vector<element>& elements,
    const std::vector<double>& displacements) {
    // One axial stress per element, in element order.
    std::vector<double> stresses;
    stresses.reserve(elements.size());

    for (const auto& el : elements) {
        const node& node_1 = nodes[el.node_1 - 1];
        const node& node_2 = nodes[el.node_2 - 1];

        // Same bar geometry as in element_stiffness().
        double delta_x = node_2.x - node_1.x;
        double delta_y = node_2.y - node_1.y;
        double length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        double cos_theta = delta_x / length;
        double sin_theta = delta_y / length;

        // Global DOF indices of this element, order [u1x, u1y, u2x, u2y].
        int global_dofs[4] = {2 * (el.node_1 - 1), 2 * (el.node_1 - 1) + 1,
                              2 * (el.node_2 - 1), 2 * (el.node_2 - 1) + 1};

        // The four nodal displacements of this element, gathered from the
        // global displacement vector.
        double u_element[4] = {
            displacements[global_dofs[0]], displacements[global_dofs[1]],
            displacements[global_dofs[2]], displacements[global_dofs[3]]};

        // Change in length = (displacement of node 2 - displacement of
        // node 1) projected onto the bar axis. Divide by L for strain.
        double change_in_length =
            -cos_theta * u_element[0] - sin_theta * u_element[1] +
            cos_theta * u_element[2] + sin_theta * u_element[3];
        double axial_strain = change_in_length / length;

        // Hooke's law: stress = E * strain.
        stresses.push_back(el.youngs_modulus * axial_strain);
    }
    return stresses;
}
