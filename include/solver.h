/**
 * @file solver.h
 * @brief Direct stiffness method routines for 2D trusses.
 *
 * Global degree-of-freedom (DOF) numbering: node @c i (1-based) maps to DOF
 * @c 2*(i-1) for x and @c 2*(i-1)+1 for y.
 */
#pragma once

#include <array>
#include <vector>

#include "types.h"

/// @brief Dense row-major matrix, stored as a vector of rows.
using matrix = std::vector<std::vector<double>>;

/**
 * @brief Element stiffness matrix of a bar in global coordinates.
 *
 * @param start_node     First node of the bar.
 * @param end_node       Second node of the bar.
 * @param area           Cross-sectional area.
 * @param youngs_modulus Young's modulus.
 * @return 4x4 matrix ordered (u_x1, u_y1, u_x2, u_y2).
 */
std::array<std::array<double, 4>, 4> k_local(const node& start_node,
                                             const node& end_node, double area,
                                             double youngs_modulus);

/**
 * @brief Assemble the global stiffness matrix from all elements.
 *
 * @param nodes    All nodes of the model.
 * @param elements Elements to assemble; node IDs are 1-based.
 * @return Square matrix of size 2 * nodes.size().
 */
matrix assemble(const std::vector<node>& nodes,
                const std::vector<elem>& elements);

/**
 * @brief Build the global load vector from point loads.
 *
 * Loads on the same DOF are summed.
 *
 * @param forces   Applied point loads.
 * @param num_dofs Total number of global DOFs.
 * @return Load vector of length @p num_dofs.
 */
std::vector<double> build_f(const std::vector<force>& forces, int num_dofs);

/**
 * @brief Impose prescribed displacements on a linear system in place.
 *
 * For each constraint, moves @c stiffness[:,dof] * displacement to the
 * right-hand side, zeroes the DOF's row and column, puts 1 on the diagonal,
 * and sets the right-hand side entry to the prescribed value.
 *
 * @param[in,out] stiffness           Global stiffness matrix to modify.
 * @param[in,out] load_vector         Global load vector to modify.
 * @param[in]     boundary_conditions Prescribed displacements to apply.
 */
void apply_bc(matrix& stiffness, std::vector<double>& load_vector,
              const std::vector<bc>& boundary_conditions);

/**
 * @brief Solve a dense linear system by Gaussian elimination with partial
 *        pivoting.
 *
 * Arguments are taken by value, so the caller's data is not modified.
 *
 * @param coefficients Square coefficient matrix.
 * @param rhs          Right-hand side vector.
 * @return Solution vector.
 * @throws std::runtime_error If a pivot is below 1e-12 in magnitude
 *         (singular system, usually an under-constrained truss).
 */
std::vector<double> gauss_solve(matrix coefficients, std::vector<double> rhs);

/**
 * @brief Compute nodal reaction forces, R = K u - F.
 *
 * @param global_stiffness Global stiffness matrix @b before boundary
 *                         conditions were applied.
 * @param displacements    Solved global displacement vector.
 * @param applied_loads    Applied global load vector.
 * @return Reaction vector; entries at unconstrained DOFs are ~0.
 */
std::vector<double> reactions(const matrix& global_stiffness,
                              const std::vector<double>& displacements,
                              const std::vector<double>& applied_loads);

/**
 * @brief Axial stress in each element from nodal displacements.
 *
 * @param nodes         All nodes of the model.
 * @param elements      Elements to evaluate.
 * @param displacements Global displacement vector.
 * @return One stress per element, in element order; positive is tension.
 */
std::vector<double> elem_stress(const std::vector<node>& nodes,
                                const std::vector<elem>& elements,
                                const std::vector<double>& displacements);
