/**
 * @file solver.cpp
 * @brief Implementation of the direct stiffness method routines.
 */
#include "solver.h"

#include <cmath>
#include <stdexcept>

std::array<std::array<double, 4>, 4> k_local(const node& start_node,
                                             const node& end_node, double area,
                                             double youngs_modulus) {
    /// Bar projection on the x axis.
    double delta_x = end_node.x - start_node.x;
    /// Bar projection on the y axis.
    double delta_y = end_node.y - start_node.y;
    /// Bar length.
    double length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    /// Direction cosine with the x axis.
    double cos_theta = delta_x / length;
    /// Direction cosine with the y axis.
    double sin_theta = delta_y / length;
    /// Axial stiffness EA/L.
    double axial_stiffness = youngs_modulus * area / length;

    /// Element stiffness matrix ordered (u_x1, u_y1, u_x2, u_y2).
    std::array<std::array<double, 4>, 4> element_stiffness = {
        {{axial_stiffness * cos_theta * cos_theta,
          axial_stiffness * cos_theta * sin_theta,
          -axial_stiffness * cos_theta * cos_theta,
          -axial_stiffness * cos_theta * sin_theta},
         {axial_stiffness * cos_theta * sin_theta,
          axial_stiffness * sin_theta * sin_theta,
          -axial_stiffness * cos_theta * sin_theta,
          -axial_stiffness * sin_theta * sin_theta},
         {-axial_stiffness * cos_theta * cos_theta,
          -axial_stiffness * cos_theta * sin_theta,
          axial_stiffness * cos_theta * cos_theta,
          axial_stiffness * cos_theta * sin_theta},
         {-axial_stiffness * cos_theta * sin_theta,
          -axial_stiffness * sin_theta * sin_theta,
          axial_stiffness * cos_theta * sin_theta,
          axial_stiffness * sin_theta * sin_theta}}};
    return element_stiffness;
}

matrix assemble(const std::vector<node>& nodes,
                const std::vector<elem>& elements) {
    /// Total number of global DOFs (2 per node).
    int num_dofs = 2 * (int)nodes.size();
    /// Global stiffness matrix being accumulated.
    matrix global_stiffness(num_dofs, std::vector<double>(num_dofs, 0.0));

    for (const auto& element : elements) {
        /// Stiffness of the current element in global coordinates.
        auto element_stiffness =
            k_local(nodes[element.start_node - 1], nodes[element.end_node - 1],
                    element.area, element.youngs_modulus);
        /// Global DOF indices of the element's four local DOFs.
        int global_dofs[4] = {
            2 * (element.start_node - 1), 2 * (element.start_node - 1) + 1,
            2 * (element.end_node - 1), 2 * (element.end_node - 1) + 1};
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                global_stiffness[global_dofs[row]][global_dofs[col]] +=
                    element_stiffness[row][col];
            }
        }
    }
    return global_stiffness;
}

std::vector<double> build_f(const std::vector<force>& forces, int num_dofs) {
    /// Global load vector being accumulated.
    std::vector<double> load_vector(num_dofs, 0.0);
    for (const auto& point_load : forces) {
        /// Global DOF index the load acts on.
        int global_dof =
            2 * (point_load.node_id - 1) + (point_load.direction - 1);
        load_vector[global_dof] += point_load.magnitude;
    }
    return load_vector;
}

void apply_bc(matrix& stiffness, std::vector<double>& load_vector,
              const std::vector<bc>& boundary_conditions) {
    /// Total number of global DOFs.
    int num_dofs = (int)load_vector.size();
    for (const auto& constraint : boundary_conditions) {
        /// Global DOF index being constrained.
        int global_dof =
            2 * (constraint.node_id - 1) + (constraint.direction - 1);
        for (int index = 0; index < num_dofs; ++index) {
            load_vector[index] -=
                stiffness[index][global_dof] * constraint.displacement;
        }
        for (int index = 0; index < num_dofs; ++index) {
            stiffness[global_dof][index] = 0.0;
            stiffness[index][global_dof] = 0.0;
        }
        stiffness[global_dof][global_dof] = 1.0;
        load_vector[global_dof] = constraint.displacement;
    }
}

std::vector<double> gauss_solve(matrix coefficients, std::vector<double> rhs) {
    /// Size of the linear system.
    int num_equations = (int)rhs.size();

    for (int pivot = 0; pivot < num_equations; ++pivot) {
        /// Row holding the largest-magnitude entry in the pivot column.
        int pivot_row = pivot;
        /// Magnitude of the best pivot candidate found so far.
        double pivot_magnitude = std::fabs(coefficients[pivot][pivot]);
        for (int row = pivot + 1; row < num_equations; ++row) {
            if (std::fabs(coefficients[row][pivot]) > pivot_magnitude) {
                pivot_magnitude = std::fabs(coefficients[row][pivot]);
                pivot_row = row;
            }
        }
        if (pivot_row != pivot) {
            std::swap(coefficients[pivot], coefficients[pivot_row]);
            std::swap(rhs[pivot], rhs[pivot_row]);
        }
        if (std::fabs(coefficients[pivot][pivot]) < 1e-12) {
            throw std::runtime_error(
                "singular stiffness matrix - check boundary conditions");
        }

        for (int row = pivot + 1; row < num_equations; ++row) {
            /// Multiple of the pivot row subtracted from this row.
            double elimination_factor =
                coefficients[row][pivot] / coefficients[pivot][pivot];
            for (int col = pivot; col < num_equations; ++col) {
                coefficients[row][col] -=
                    elimination_factor * coefficients[pivot][col];
            }
            rhs[row] -= elimination_factor * rhs[pivot];
        }
    }

    /// Solution vector filled by back substitution.
    std::vector<double> solution(num_equations, 0.0);
    for (int row = num_equations - 1; row >= 0; --row) {
        /// Right-hand side minus contributions of already-solved unknowns.
        double remainder = rhs[row];
        for (int col = row + 1; col < num_equations; ++col) {
            remainder -= coefficients[row][col] * solution[col];
        }
        solution[row] = remainder / coefficients[row][row];
    }
    return solution;
}

std::vector<double> reactions(const matrix& global_stiffness,
                              const std::vector<double>& displacements,
                              const std::vector<double>& applied_loads) {
    /// Total number of global DOFs.
    int num_dofs = (int)displacements.size();
    /// Reaction force at each global DOF.
    std::vector<double> reaction_forces(num_dofs, 0.0);
    for (int row = 0; row < num_dofs; ++row) {
        /// Internal nodal force (row of K u) at this DOF.
        double internal_force = 0.0;
        for (int col = 0; col < num_dofs; ++col) {
            internal_force += global_stiffness[row][col] * displacements[col];
        }
        reaction_forces[row] = internal_force - applied_loads[row];
    }
    return reaction_forces;
}

std::vector<double> elem_stress(const std::vector<node>& nodes,
                                const std::vector<elem>& elements,
                                const std::vector<double>& displacements) {
    /// Axial stress per element, in element order.
    std::vector<double> axial_stresses;
    axial_stresses.reserve(elements.size());

    for (const auto& element : elements) {
        const node& start_node = nodes[element.start_node - 1];
        const node& end_node = nodes[element.end_node - 1];
        /// Bar projection on the x axis.
        double delta_x = end_node.x - start_node.x;
        /// Bar projection on the y axis.
        double delta_y = end_node.y - start_node.y;
        /// Bar length.
        double length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        /// Direction cosine with the x axis.
        double cos_theta = delta_x / length;
        /// Direction cosine with the y axis.
        double sin_theta = delta_y / length;

        /// Global DOF indices of the element's four local DOFs.
        int global_dofs[4] = {
            2 * (element.start_node - 1), 2 * (element.start_node - 1) + 1,
            2 * (element.end_node - 1), 2 * (element.end_node - 1) + 1};
        /// Displacements of the element's four DOFs.
        double element_displacements[4] = {
            displacements[global_dofs[0]], displacements[global_dofs[1]],
            displacements[global_dofs[2]], displacements[global_dofs[3]]};

        /// Elongation along the bar axis divided by length.
        double axial_strain = (-cos_theta * element_displacements[0] -
                               sin_theta * element_displacements[1] +
                               cos_theta * element_displacements[2] +
                               sin_theta * element_displacements[3]) /
                              length;
        axial_stresses.push_back(element.youngs_modulus * axial_strain);
    }
    return axial_stresses;
}
