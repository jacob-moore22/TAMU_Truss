#pragma once

#include <array>
#include <vector>

#include "types.h"

/// Dense global stiffness (or working) matrix, row-major, sized 2N x 2N for N
/// nodes.
using matrix = std::vector<std::vector<double>>;

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
                                             double cross_section_area, double youngs_modulus);

/**
 * @brief Assemble the global stiffness matrix from all elements.
 *
 * @param nodes All node positions in the model.
 * @param elements All truss elements in the model.
 * @return The assembled global stiffness matrix, sized 2*nodes.size() square.
 */
matrix assemble(const std::vector<node>& nodes, const std::vector<elem>& elements);

/**
 * @brief Build the global applied-force vector from a list of point forces.
 *
 * @param forces Applied forces to scatter into the vector.
 * @param dof_count Total number of degrees of freedom (2 * node count).
 * @return A force vector of length dof_count.
 */
std::vector<double> build_f(const std::vector<force>& forces, int dof_count);

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
              const std::vector<bc>& boundary_conditions);

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
std::vector<double> gauss_solve(matrix stiffness_matrix, std::vector<double> force_vector);

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
                              const std::vector<double>& applied_forces);

/**
 * @brief Compute axial stress in every truss element from nodal displacements.
 *
 * @param nodes All node positions in the model.
 * @param elements All truss elements in the model.
 * @param displacements Solved displacement vector.
 * @return Axial stress for each element, in the same order as @p elements.
 */
std::vector<double> elem_stress(const std::vector<node>& nodes, const std::vector<elem>& elements,
                                const std::vector<double>& displacements);
