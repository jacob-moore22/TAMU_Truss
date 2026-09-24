/**
 * @file types.h
 * @brief Data structures describing a 2D truss.
 *
 * Conventions used throughout the solver:
 *  - Nodes and elements are numbered from 1 in the input file (as an engineer
 *    would number them on a drawing). Internally, vectors are 0-based, so
 *    node @f$N@f$ is stored at index @f$N-1@f$.
 *  - Every node has two degrees of freedom (DOFs): displacement in X and in
 *    Y. The global DOF index of node @f$N@f$ is @f$2(N-1)@f$ for X and
 *    @f$2(N-1)+1@f$ for Y.
 *  - Units are whatever the input file uses, as long as they are consistent
 *    (e.g. metres, newtons, pascals).
 */
#pragma once

#include <vector>

/// @brief A joint of the truss, located at (x, y).
struct node {
    double x;  ///< X coordinate
    double y;  ///< Y coordinate
};

/// @brief A straight two-force bar connecting two nodes.
struct element {
    int node_1;             ///< 1-based ID of the first end node
    int node_2;             ///< 1-based ID of the second end node
    double area;            ///< Cross-sectional area @f$A@f$
    double youngs_modulus;  ///< Young's modulus @f$E@f$ of the material
};

/**
 * @brief A prescribed displacement (support) at one DOF of one node.
 *
 * A @ref value of 0 is a fixed support; a non-zero value is a settlement.
 */
struct boundary_condition {
    int node_id;   ///< 1-based node the support acts on
    int dof;       ///< Direction: 1 = X, 2 = Y
    double value;  ///< Prescribed displacement in that direction
};

/// @brief A concentrated force applied at one DOF of one node.
struct force {
    int node_id;   ///< 1-based node the force acts on
    int dof;       ///< Direction: 1 = X, 2 = Y
    double value;  ///< Force magnitude (sign gives direction along the axis)
};

/// @brief The complete truss problem read from the input file.
struct model {
    std::vector<node> nodes;                   ///< Node N is `nodes[N-1]`
    std::vector<element> elements;             ///< In input-file order
    std::vector<boundary_condition> supports;  ///< All prescribed displacements
    std::vector<force> loads;                  ///< All applied forces
    int num_load_steps = 1;  ///< Loads are ramped 0 → full over this many steps
};
