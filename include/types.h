#pragma once

#include <vector>

struct node {
    double x;
    double y;
};

struct elem {
    int n1;
    int n2;
    double a;
    double e;
};

struct bc {
    int node;
    int dof;
    double val;
};

struct force {
    int node;
    int dof;
    double val;
};

struct model {
    std::vector<node> nodes;
    std::vector<elem> elems;
    std::vector<bc> bcs;
    std::vector<force> forces;
    int load_steps = 1;
};
