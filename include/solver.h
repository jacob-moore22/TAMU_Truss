/**
 * \file solver.h
 * \brief Direct stiffness method FEM solver for 2D pin-jointed trusses:
 *        element stiffness, global assembly, boundary conditions, a
 *        hand-rolled Gaussian elimination solve, reactions, and axial
 *        stress recovery.
 */
#pragma once

#include "types.h"
#include <array>
#include <vector>

/// Dense global stiffness / system matrix, indexed `matrix[row][col]`.
using matrix = std::vector<std::vector<double>>;

/**
 * \brief Compute the 4x4 local stiffness matrix for a single 2-node axial
 *        element in global coordinates.
 * \param a Position of the element's first node.
 * \param b Position of the element's second node.
 * \param area Cross-sectional area of the element.
 * \param e Young's modulus of the element.
 * \return The 4x4 element stiffness matrix, ordered [a.x, a.y, b.x, b.y].
 */
std::array<std::array<double, 4>, 4> k_local(const node &a, const node &b, double area, double e);

/**
 * \brief Assemble the global stiffness matrix by scattering each element's
 *        local stiffness (see k_local()) into its global degrees of freedom.
 * \param nodes All nodes in the model.
 * \param elems All elements in the model.
 * \return The `2*nodes.size()` x `2*nodes.size()` global stiffness matrix.
 */
matrix assemble(const std::vector<node> &nodes, const std::vector<elem> &elems);

/**
 * \brief Build the global load vector from a list of point forces.
 * \param forces Applied point loads.
 * \param n_dof Total number of degrees of freedom (`2 * node count`).
 * \return Load vector of length \p n_dof; forces on the same DOF accumulate.
 */
std::vector<double> build_f(const std::vector<force> &forces, int n_dof);

/**
 * \brief Apply displacement boundary conditions in place, using row/column
 *        elimination: each constrained DOF's row/column is zeroed, its
 *        diagonal set to 1, and the load vector adjusted so the system
 *        remains consistent.
 * \param k_global Global stiffness matrix, modified in place.
 * \param f_vec Global load vector, modified in place.
 * \param bcs Boundary conditions to apply.
 */
void apply_bc(matrix &k_global, std::vector<double> &f_vec, const std::vector<bc> &bcs);

/**
 * \brief Solve `k_global * u = f_vec` for `u` via Gaussian elimination with
 *        partial pivoting. Both arguments are taken by value since the
 *        elimination modifies them in place internally.
 * \param k_global System matrix (already boundary-condition-adjusted).
 * \param f_vec Load vector (already boundary-condition-adjusted).
 * \return The displacement vector `u`.
 * \throws std::runtime_error if the matrix is singular (a pivot magnitude
 *         falls below `1e-12`), typically from insufficient/incorrect
 *         boundary conditions.
 */
std::vector<double> gauss_solve(matrix k_global, std::vector<double> f_vec);

/**
 * \brief Compute reaction/residual forces `R = K*u - F` for the
 *        unconstrained ("original") system.
 * \param k_global The original (pre-boundary-condition) global stiffness matrix.
 * \param u_vec Solved displacement vector.
 * \param f_vec The original (pre-boundary-condition) load vector.
 * \return Residual force vector; nonzero only at constrained DOFs in a
 *         correctly-solved system, giving the support reactions there.
 */
std::vector<double> reactions(const matrix &k_global, const std::vector<double> &u_vec,
                              const std::vector<double> &f_vec);

/**
 * \brief Recover axial stress in each element from the solved displacement
 *        field.
 * \param nodes All nodes in the model.
 * \param elems All elements in the model.
 * \param u_vec Solved displacement vector.
 * \return Axial stress per element, in the same order as \p elems.
 */
std::vector<double> elem_stress(const std::vector<node> &nodes, const std::vector<elem> &elems,
                                const std::vector<double> &u_vec);
