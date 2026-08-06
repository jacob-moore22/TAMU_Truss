/**
 * \file types.h
 * \brief Core data structures for the truss model: geometry, elements,
 *        boundary conditions, applied loads, and the model that ties them
 *        together.
 *
 * These map directly onto the `*NODES`/`*ELEMENTS`/`*BOUNDARIES`/`*FORCES`/
 * `*LOAD_STEPS` sections of the plain-text input format (see io.h).
 */
#pragma once

#include <vector>

/// A 2D node position.
struct node {
    double x; ///< X coordinate.
    double y; ///< Y coordinate.
};

/// A 2-node truss (axial) element connecting two nodes.
struct elem {
    int n1;    ///< 1-based ID of the first node.
    int n2;    ///< 1-based ID of the second node.
    double a;  ///< Cross-sectional area.
    double e;  ///< Young's modulus.
};

/// A displacement boundary condition applied to one degree of freedom.
struct bc {
    int node;    ///< 1-based node ID the constraint is applied to.
    int dof;     ///< Degree of freedom: 1 = X, 2 = Y.
    double val;  ///< Prescribed displacement value.
};

/// A point load applied to one degree of freedom.
struct force {
    int node;    ///< 1-based node ID the load is applied to.
    int dof;     ///< Degree of freedom: 1 = X, 2 = Y.
    double val;  ///< Load magnitude.
};

/// The full truss model as parsed from an input file.
struct model {
    std::vector<node> nodes;    ///< All nodes, indexed by (1-based ID - 1).
    std::vector<elem> elems;    ///< All elements.
    std::vector<bc> bcs;        ///< All boundary conditions.
    std::vector<force> forces;  ///< All applied loads (at full magnitude).
    int load_steps = 1;         ///< Number of load steps to ramp forces over.
};
