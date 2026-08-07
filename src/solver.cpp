#include "solver.h"

#include <cmath>
#include <stdexcept>

/**
 * @brief Compute the 4x4 local stiffness matrix for a single 2D truss element.
 *
 * @param node_a First endpoint of the element.
 * @param node_b Second endpoint of the element.
 * @param cross_section_area Cross-sectional area of the element.
 * @param youngs_modulus Young's modulus of the element material.
 * @return The local stiffness matrix in global (x, y) coordinates, ordered
 *         [node_a.x, node_a.y, node_b.x, node_b.y].
 */
std::array<std::array<double, 4>, 4> k_local(const node& node_a, const node& node_b,
                                             double cross_section_area, double youngs_modulus) {
    double delta_x = node_b.x - node_a.x;
    double delta_y = node_b.y - node_a.y;
    double length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    double cos_theta = delta_x / length;
    double sin_theta = delta_y / length;
    double axial_stiffness = youngs_modulus * cross_section_area / length;

    std::array<std::array<double, 4>, 4> local_stiffness = {
        {{axial_stiffness * cos_theta * cos_theta, axial_stiffness * cos_theta * sin_theta,
          -axial_stiffness * cos_theta * cos_theta, -axial_stiffness * cos_theta * sin_theta},
         {axial_stiffness * cos_theta * sin_theta, axial_stiffness * sin_theta * sin_theta,
          -axial_stiffness * cos_theta * sin_theta, -axial_stiffness * sin_theta * sin_theta},
         {-axial_stiffness * cos_theta * cos_theta, -axial_stiffness * cos_theta * sin_theta,
          axial_stiffness * cos_theta * cos_theta, axial_stiffness * cos_theta * sin_theta},
         {-axial_stiffness * cos_theta * sin_theta, -axial_stiffness * sin_theta * sin_theta,
          axial_stiffness * cos_theta * sin_theta, axial_stiffness * sin_theta * sin_theta}}};
    return local_stiffness;
}

/**
 * @brief Assemble the global stiffness matrix from all elements.
 *
 * @param nodes All node positions in the model.
 * @param elements All truss elements in the model.
 * @return The assembled global stiffness matrix, sized 2*nodes.size() square.
 */
matrix assemble(const std::vector<node>& nodes, const std::vector<elem>& elements) {
    int dof_count = 2 * (int)nodes.size();
    matrix global_stiffness(dof_count, std::vector<double>(dof_count, 0.0));

    for (const auto& element : elements) {
        auto local_stiffness = k_local(nodes[element.node1_id - 1], nodes[element.node2_id - 1],
                                       element.cross_section_area, element.youngs_modulus);
        int dof_indices[4] = {2 * (element.node1_id - 1), 2 * (element.node1_id - 1) + 1,
                              2 * (element.node2_id - 1), 2 * (element.node2_id - 1) + 1};
        for (int local_row = 0; local_row < 4; ++local_row) {
            for (int local_col = 0; local_col < 4; ++local_col) {
                global_stiffness[dof_indices[local_row]][dof_indices[local_col]] +=
                    local_stiffness[local_row][local_col];
            }
        }
    }
    return global_stiffness;
}

/**
 * @brief Build the global applied-force vector from a list of point forces.
 *
 * @param forces Applied forces to scatter into the vector.
 * @param dof_count Total number of degrees of freedom (2 * node count).
 * @return A force vector of length dof_count.
 */
std::vector<double> build_f(const std::vector<force>& forces, int dof_count) {
    std::vector<double> global_force(dof_count, 0.0);
    for (const auto& applied_force : forces) {
        int dof_index = 2 * (applied_force.node_id - 1) + (applied_force.dof_index - 1);
        global_force[dof_index] += applied_force.value;
    }
    return global_force;
}

/**
 * @brief Apply displacement boundary conditions to a stiffness matrix and force
 * vector.
 *
 * Eliminates each constrained degree of freedom by zeroing its row and column
 * in @p stiffness_matrix, setting the diagonal to 1, and correcting
 * @p force_vector so the system still solves to the prescribed value at that
 * DOF.
 *
 * @param[in,out] stiffness_matrix Global stiffness matrix, modified in place.
 * @param[in,out] force_vector Global force vector, modified in place.
 * @param boundary_conditions Prescribed displacement boundary conditions to
 * apply.
 */
void apply_bc(matrix& stiffness_matrix, std::vector<double>& force_vector,
              const std::vector<bc>& boundary_conditions) {
    int dof_count = (int)force_vector.size();
    for (const auto& boundary_condition : boundary_conditions) {
        int dof_index = 2 * (boundary_condition.node_id - 1) + (boundary_condition.dof_index - 1);
        for (int row = 0; row < dof_count; ++row) {
            force_vector[row] -= stiffness_matrix[row][dof_index] * boundary_condition.value;
        }
        for (int row = 0; row < dof_count; ++row) {
            stiffness_matrix[dof_index][row] = 0.0;
            stiffness_matrix[row][dof_index] = 0.0;
        }
        stiffness_matrix[dof_index][dof_index] = 1.0;
        force_vector[dof_index] = boundary_condition.value;
    }
}

/**
 * @brief Solve a linear system via Gaussian elimination with partial pivoting.
 *
 * @param stiffness_matrix Global stiffness matrix, with boundary conditions
 * applied.
 * @param force_vector Global force vector, with boundary conditions applied.
 * @return The solved displacement vector.
 * @throws std::runtime_error if the matrix is singular (within 1e-12
 * tolerance).
 */
std::vector<double> gauss_solve(matrix stiffness_matrix, std::vector<double> force_vector) {
    int dof_count = (int)force_vector.size();

    for (int pivot_row = 0; pivot_row < dof_count; ++pivot_row) {
        int max_row = pivot_row;
        double max_val = std::fabs(stiffness_matrix[pivot_row][pivot_row]);
        for (int row = pivot_row + 1; row < dof_count; ++row) {
            if (std::fabs(stiffness_matrix[row][pivot_row]) > max_val) {
                max_val = std::fabs(stiffness_matrix[row][pivot_row]);
                max_row = row;
            }
        }
        if (max_row != pivot_row) {
            std::swap(stiffness_matrix[pivot_row], stiffness_matrix[max_row]);
            std::swap(force_vector[pivot_row], force_vector[max_row]);
        }
        if (std::fabs(stiffness_matrix[pivot_row][pivot_row]) < 1e-12) {
            throw std::runtime_error("singular stiffness matrix - check boundary conditions");
        }

        for (int row = pivot_row + 1; row < dof_count; ++row) {
            double elimination_factor =
                stiffness_matrix[row][pivot_row] / stiffness_matrix[pivot_row][pivot_row];
            for (int col = pivot_row; col < dof_count; ++col) {
                stiffness_matrix[row][col] -= elimination_factor * stiffness_matrix[pivot_row][col];
            }
            force_vector[row] -= elimination_factor * force_vector[pivot_row];
        }
    }

    std::vector<double> displacements(dof_count, 0.0);
    for (int row = dof_count - 1; row >= 0; --row) {
        double residual = force_vector[row];
        for (int col = row + 1; col < dof_count; ++col) {
            residual -= stiffness_matrix[row][col] * displacements[col];
        }
        displacements[row] = residual / stiffness_matrix[row][row];
    }
    return displacements;
}

/**
 * @brief Compute reaction forces at every degree of freedom.
 *
 * @param stiffness_matrix The original (pre-boundary-condition) global
 * stiffness matrix.
 * @param displacements Solved displacement vector.
 * @param applied_forces Applied force vector, before boundary conditions were
 * applied.
 * @return Reaction force at every degree of freedom (K * u - f).
 */
std::vector<double> reactions(const matrix& stiffness_matrix, const std::vector<double>& displacements,
                              const std::vector<double>& applied_forces) {
    int dof_count = (int)displacements.size();
    std::vector<double> reaction_forces(dof_count, 0.0);
    for (int row = 0; row < dof_count; ++row) {
        double stiffness_times_displacement = 0.0;
        for (int col = 0; col < dof_count; ++col) {
            stiffness_times_displacement += stiffness_matrix[row][col] * displacements[col];
        }
        reaction_forces[row] = stiffness_times_displacement - applied_forces[row];
    }
    return reaction_forces;
}

/**
 * @brief Compute axial stress in every truss element from nodal displacements.
 *
 * @param nodes All node positions in the model.
 * @param elements All truss elements in the model.
 * @param displacements Solved displacement vector.
 * @return Axial stress for each element, in the same order as @p elements.
 */
std::vector<double> elem_stress(const std::vector<node>& nodes, const std::vector<elem>& elements,
                                const std::vector<double>& displacements) {
    std::vector<double> stresses;
    stresses.reserve(elements.size());

    for (const auto& element : elements) {
        const node& node_a = nodes[element.node1_id - 1];
        const node& node_b = nodes[element.node2_id - 1];
        double delta_x = node_b.x - node_a.x;
        double delta_y = node_b.y - node_a.y;
        double length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        double cos_theta = delta_x / length;
        double sin_theta = delta_y / length;

        int dof_indices[4] = {2 * (element.node1_id - 1), 2 * (element.node1_id - 1) + 1,
                              2 * (element.node2_id - 1), 2 * (element.node2_id - 1) + 1};
        double element_displacements[4] = {displacements[dof_indices[0]], displacements[dof_indices[1]],
                                           displacements[dof_indices[2]], displacements[dof_indices[3]]};

        double strain = (-cos_theta * element_displacements[0] - sin_theta * element_displacements[1] +
                         cos_theta * element_displacements[2] + sin_theta * element_displacements[3]) /
                        length;
        stresses.push_back(element.youngs_modulus * strain);
    }
    return stresses;
}
