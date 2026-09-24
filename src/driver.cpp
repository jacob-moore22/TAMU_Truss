/**
 * @file driver.cpp
 * @brief Implementation of the load-stepping driver (see driver.h).
 */
#include "driver.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "solver.h"
#include "vtk_writer.h"

namespace {

/**
 * @brief Builds the file name for one load step.
 *
 * @param output_dir Directory the file goes in.
 * @param step       Load-step number.
 * @param num_digits Zero-padding width, so files sort correctly.
 * @return `<output_dir>/results_step_NN.vtk`
 */
std::string step_filename(const std::string& output_dir, int step,
                          int num_digits) {
    std::ostringstream name;
    name << output_dir << "/results_step_" << std::setfill('0')
         << std::setw(num_digits) << step << ".vtk";
    return name.str();
}

}  // namespace

void run_solver(const model& truss, const std::string& output_dir) {
    std::string final_path = output_dir + "/results.vtk";
    int num_dofs = 2 * (int)truss.nodes.size();

    // Stiffness and loads are independent of the load step, so build them
    // once. k_global is kept unmodified for the reaction calculation.
    matrix k_global = assemble_global_stiffness(truss.nodes, truss.elements);
    std::vector<double> full_load_vector =
        build_load_vector(truss.loads, num_dofs);

    // Loads are ramped from 0 to full value over this many steps.
    int num_steps = truss.num_load_steps > 0 ? truss.num_load_steps : 1;
    // Zero-padding width for the step number in file names (at least 2).
    int num_digits = std::max((int)std::to_string(num_steps).size(), 2);

    // Results of the most recent step; reused for the final results.vtk.
    std::vector<double> displacements;    // {u}, 2 per node
    std::vector<double> reaction_forces;  // {R}, 2 per node
    std::vector<double> stresses;         // 1 per element

    // Step 0: the undeformed truss with no load.
    std::vector<double> zero_displacements(num_dofs, 0.0);
    std::vector<double> zero_stresses(truss.elements.size(), 0.0);
    write_vtk(step_filename(output_dir, 0, num_digits), truss.nodes,
              truss.elements, zero_displacements, zero_stresses);

    for (int step = 1; step <= num_steps; ++step) {
        // Fraction of the full load applied in this step (1.0 at the end).
        double load_factor = (double)step / num_steps;

        std::vector<double> step_load_vector(num_dofs);
        for (int i = 0; i < num_dofs; ++i) {
            step_load_vector[i] = full_load_vector[i] * load_factor;
        }

        // apply_boundary_conditions modifies its arguments, so work on
        // copies and keep the originals for the reactions.
        matrix k_constrained = k_global;
        std::vector<double> constrained_load_vector = step_load_vector;
        apply_boundary_conditions(k_constrained, constrained_load_vector,
                                  truss.supports);
        displacements = gauss_solve(k_constrained, constrained_load_vector);

        reaction_forces =
            compute_reactions(k_global, displacements, step_load_vector);
        stresses = compute_element_stresses(truss.nodes, truss.elements,
                                            displacements);

        write_vtk(step_filename(output_dir, step, num_digits), truss.nodes,
                  truss.elements, displacements, stresses);
    }

    write_vtk(final_path, truss.nodes, truss.elements, displacements, stresses);
    std::cout
        << "wrote " << (num_steps + 1)
        << " load-step file(s) (results_step_*.vtk, incl. step 0) and final "
        << final_path << "\n";
}
