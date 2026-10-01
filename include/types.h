/**
 * @file types.h
 * @brief Plain data types describing a 2D truss model.
 *
 * All node references inside these structs are 1-based IDs as they appear in
 * the input file. Node @c i owns global degrees of freedom @c 2*(i-1) (x) and
 * @c 2*(i-1)+1 (y).
 */
#pragma once

#include <vector>

/// @brief A truss joint located in the 2D plane.
struct node {
    double x;  ///< x coordinate of the node.
    double y;  ///< y coordinate of the node.
};

/// @brief A two-node axial bar element connecting two nodes.
struct elem {
    int start_node;         ///< 1-based ID of the element's first node.
    int end_node;           ///< 1-based ID of the element's second node.
    double area;            ///< Cross-sectional area of the bar.
    double youngs_modulus;  ///< Young's modulus (elastic modulus) of the bar.
};

/// @brief A prescribed displacement on one degree of freedom of a node.
struct bc {
    int node_id;          ///< 1-based ID of the constrained node.
    int direction;        ///< Constrained direction: 1 = x, 2 = y.
    double displacement;  ///< Prescribed displacement value (0 for a support).
};

/// @brief A point load applied to one degree of freedom of a node.
struct force {
    int node_id;       ///< 1-based ID of the loaded node.
    int direction;     ///< Load direction: 1 = x, 2 = y.
    double magnitude;  ///< Full (final load step) value of the applied force.
};

/// @brief Complete truss problem definition as read from an input file.
struct model {
    std::vector<node> nodes;              ///< Nodes, indexed by ID - 1.
    std::vector<elem> elements;           ///< Bar elements, in file order.
    std::vector<bc> boundary_conditions;  ///< Prescribed displacements.
    std::vector<force> forces;            ///< Applied point loads.
    int load_steps = 1;  ///< Number of increments used to ramp the loads.
};
