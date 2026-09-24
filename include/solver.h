/**
 * @file solver.h
 * @brief Direct stiffness method for 2D trusses.
 *
 * The global system solved is
 * @f[ [K]\{u\} = \{F\} @f]
 * where
 *  - @f$[K]@f$ is the global stiffness matrix (num_dofs × num_dofs),
 *  - @f$\{u\}@f$ is the vector of nodal displacements (one per DOF), and
 *  - @f$\{F\}@f$ is the vector of applied nodal forces (one per DOF).
 *
 * See types.h for how node numbers map to DOF indices.
 */
#pragma once

#include <array>
#include <vector>

#include "types.h"

/// @brief A dense square matrix stored as a vector of rows: `k[row][col]`.
using matrix = std::vector<std::vector<double>>;

/**
 * @brief A 4×4 element stiffness matrix in global coordinates.
 *
 * Rows and columns are ordered @f$[u_{1x}, u_{1y}, u_{2x}, u_{2y}]@f$ for the
 * two end nodes of the element.
 */
using element_matrix = std::array<std::array<double, 4>, 4>;

/**
 * @brief Builds the stiffness matrix of one bar in global (X, Y) coordinates.
 *
 * @f[
 * [k_e] = \frac{EA}{L}
 * \begin{bmatrix}
 *  c^2 &  cs & -c^2 & -cs  \\
 *  cs  & s^2 & -cs  & -s^2 \\
 * -c^2 & -cs &  c^2 &  cs  \\
 * -cs  & -s^2 & cs  &  s^2
 * \end{bmatrix}
 * @f]
 * where @f$c = \cos\theta@f$, @f$s = \sin\theta@f$ and @f$\theta@f$ is the bar
 * angle measured from the global X axis.
 *
 * @param node_1         First end node.
 * @param node_2         Second end node.
 * @param area           Cross-sectional area @f$A@f$.
 * @param youngs_modulus Young's modulus @f$E@f$.
 * @return The 4×4 element stiffness matrix, ordered as in @ref element_matrix.
 */
element_matrix element_stiffness(const node& node_1, const node& node_2,
                                 double area, double youngs_modulus);

/**
 * @brief Assembles the global stiffness matrix @f$[K]@f$.
 *
 * Each element's 4×4 matrix (from element_stiffness()) is added into the
 * rows and columns of that element's four global DOFs.
 *
 * @param nodes    All nodes of the truss.
 * @param elements All elements of the truss.
 * @return The global stiffness matrix, size @f$2n \times 2n@f$ for @f$n@f$
 *         nodes.
 */
matrix assemble_global_stiffness(const std::vector<node>& nodes,
                                 const std::vector<element>& elements);

/**
 * @brief Builds the global load vector @f$\{F\}@f$.
 *
 * Forces acting on the same DOF are summed; every other entry is zero.
 *
 * @param loads    Applied nodal forces.
 * @param num_dofs Total number of DOFs (2 × number of nodes).
 * @return The load vector, one entry per DOF.
 */
std::vector<double> build_load_vector(const std::vector<force>& loads,
                                      int num_dofs);

/**
 * @brief Applies prescribed displacements to @f$[K]@f$ and @f$\{F\}@f$.
 *
 * For each constrained DOF the row and column of @f$[K]@f$ are zeroed, the
 * diagonal is set to 1, and @f$\{F\}@f$ is adjusted so that solving the
 * system returns the prescribed value at that DOF.
 *
 * @param[in,out] k_global    Global stiffness matrix; modified in place.
 * @param[in,out] load_vector Global load vector; modified in place.
 * @param[in]     supports    Prescribed displacements to enforce.
 */
void apply_boundary_conditions(matrix& k_global,
                               std::vector<double>& load_vector,
                               const std::vector<boundary_condition>& supports);

/**
 * @brief Solves @f$[K]\{u\} = \{F\}@f$ by Gaussian elimination with partial
 *        pivoting.
 *
 * Both arguments are taken by value, so the caller's data is unchanged.
 *
 * @param k_global    Coefficient matrix (with boundary conditions applied).
 * @param load_vector Right-hand side vector.
 * @return The solution vector @f$\{u\}@f$.
 * @throws std::runtime_error if a pivot is numerically zero (singular matrix,
 *         usually meaning the truss is a mechanism or under-supported).
 */
std::vector<double> gauss_solve(matrix k_global,
                                std::vector<double> load_vector);

/**
 * @brief Computes support reactions @f$\{R\} = [K]\{u\} - \{F\}@f$.
 *
 * Must be called with the @b unconstrained stiffness matrix (before
 * apply_boundary_conditions()). Entries at free DOFs should be approximately
 * zero, which is a useful equilibrium check.
 *
 * @param k_global      Unconstrained global stiffness matrix.
 * @param displacements Solved nodal displacements.
 * @param load_vector   Applied load vector (without BC modifications).
 * @return Reaction force at every DOF.
 */
std::vector<double> compute_reactions(const matrix& k_global,
                                      const std::vector<double>& displacements,
                                      const std::vector<double>& load_vector);

/**
 * @brief Computes the axial stress in each element.
 *
 * @f[ \sigma = E\,\varepsilon, \qquad
 *     \varepsilon = \frac{\Delta L}{L} @f]
 * Positive stress is tension, negative is compression.
 *
 * @param nodes         All nodes of the truss.
 * @param elements      All elements of the truss.
 * @param displacements Solved nodal displacements.
 * @return One stress value per element, in the same order as @p elements.
 */
std::vector<double> compute_element_stresses(
    const std::vector<node>& nodes, const std::vector<element>& elements,
    const std::vector<double>& displacements);
