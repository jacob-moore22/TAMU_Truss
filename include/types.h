#pragma once

#include <vector>

struct node {
    double x, y;
};

struct elem {
    int n1, n2;
    double a, e;
};

struct bc {
    int node, dof;
    double val;
};

struct force {
    int node, dof;
    double val;
};

struct model {
    std::vector<node> nodes;
    std::vector<elem> elems;
    std::vector<bc> bcs;
    std::vector<force> forces;
    int load_steps = 1;
};
