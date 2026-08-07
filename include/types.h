#pragma once

#include <vector>

/// A 2D node position in the truss geometry.
struct node {
    double x;  ///< X coordinate.
    double y;  ///< Y coordinate.
};

/// A single truss element (bar) connecting two nodes.
struct elem {
    int node1_id;               ///< 1-indexed ID of the element's first node.
    int node2_id;               ///< 1-indexed ID of the element's second node.
    double cross_section_area;  ///< Cross-sectional area of the bar.
    double youngs_modulus;      ///< Young's modulus (elastic modulus) of the bar
                                ///< material.
};

/// A prescribed displacement boundary condition at a single degree of freedom.
struct bc {
    int node_id;    ///< 1-indexed ID of the constrained node.
    int dof_index;  ///< Degree of freedom index (1 = X, 2 = Y).
    double value;   ///< Prescribed displacement value.
};

/// An applied external force at a single degree of freedom.
struct force {
    int node_id;    ///< 1-indexed ID of the loaded node.
    int dof_index;  ///< Degree of freedom index (1 = X, 2 = Y).
    double value;   ///< Applied force magnitude.
};

/// The complete truss model: geometry, connectivity, boundary conditions, and
/// loading.
struct model {
    std::vector<node> nodes;              ///< Node positions, indexed by (id - 1).
    std::vector<elem> elements;           ///< Truss elements (bars).
    std::vector<bc> boundary_conditions;  ///< Prescribed displacement boundary conditions.
    std::vector<force> forces;            ///< Applied external forces.
    int load_steps = 1;                   ///< Number of load steps to ramp the applied forces over.
};
