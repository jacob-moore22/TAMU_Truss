#pragma once

#include <array>
#include <vector>
#include "types.h"

using matrix = std::vector<std::vector<double>>;

std::array<std::array<double, 4>, 4> k_local(const node &a, const node &b, double area, double e);

matrix assemble(const std::vector<node> &nodes, const std::vector<elem> &elems);

std::vector<double> build_f(const std::vector<force> &forces, int n_dof);

void apply_bc(matrix &k_global, std::vector<double> &f_vec, const std::vector<bc> &bcs);

std::vector<double> gauss_solve(matrix k_global, std::vector<double> f_vec);

std::vector<double> reactions(const matrix &k_global, const std::vector<double> &u_vec, const std::vector<double> &f_vec);

std::vector<double> elem_stress(const std::vector<node> &nodes, const std::vector<elem> &elems, const std::vector<double> &u_vec);
