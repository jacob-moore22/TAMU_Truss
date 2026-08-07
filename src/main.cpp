#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "io.h"
#include "solver.h"
#include "vtk_writer.h"

int main(int argc, char** argv) {
    std::string in_path = argc > 1 ? argv[1] : "examples/input_triangle.txt";
    std::string out_dir = argc > 2 ? argv[2] : "results";
    std::string out_path = out_dir + "/results.vtk";

    model m = read_input(in_path);
    int n_dof = 2 * (int)m.nodes.size();

    matrix k_orig = assemble(m.nodes, m.elems);
    std::vector<double> f_full = build_f(m.forces, n_dof);

    int n_steps = m.load_steps > 0 ? m.load_steps : 1;
    int width = std::max((int)std::to_string(n_steps).size(), 2);

    std::vector<double> u_vec, r_vec, stresses;

    std::vector<double> u_zero(n_dof, 0.0);
    std::vector<double> stress_zero(m.elems.size(), 0.0);
    std::ostringstream name0;
    name0 << out_dir << "/results_step_" << std::setfill('0')
          << std::setw(width) << 0 << ".vtk";
    write_vtk(name0.str(), m.nodes, m.elems, u_zero, stress_zero);
    // clang-format off
    // std::cout << "load step 0/" << n_steps << " (factor=0, initial conditions): wrote " << name0.str() << "\n";
    // clang-format on

    for (int step = 1; step <= n_steps; ++step) {
        double factor = (double)step / n_steps;

        std::vector<double> f_step(n_dof);
        for (int i = 0; i < n_dof; ++i) f_step[i] = f_full[i] * factor;

        matrix k_step = k_orig;
        std::vector<double> f_bc = f_step;
        apply_bc(k_step, f_bc, m.bcs);
        u_vec = gauss_solve(k_step, f_bc);

        r_vec = reactions(k_orig, u_vec, f_step);
        stresses = elem_stress(m.nodes, m.elems, u_vec);

        std::ostringstream name;
        name << out_dir << "/results_step_" << std::setfill('0')
             << std::setw(width) << step << ".vtk";
        write_vtk(name.str(), m.nodes, m.elems, u_vec, stresses);
    }

    write_vtk(out_path, m.nodes, m.elems, u_vec, stresses);
    std::cout
        << "wrote " << (n_steps + 1)
        << " load-step file(s) (results_step_*.vtk, incl. step 0) and final "
        << out_path << "\n";

    return 0;
}
