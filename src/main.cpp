#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "io.h"
#include "solver.h"
#include "vtk_writer.h"

/**
 * @brief CLI entry point: run a linear-elastic truss solve and write
 * per-load-step VTK output.
 *
 * Usage: `truss_solver [input_file] [output_dir]`. Loads and solves the model
 * once per load step, ramping the applied forces from 0 to full value, and
 * writes a `results_step_NN.vtk` file per step (including an all-zero step 0)
 * plus a final `results.vtk` with the last step's solution.
 *
 * @param argc Argument count.
 * @param argv Argument vector: `argv[1]` is the input file (default
 *             `examples/input_triangle.txt`), `argv[2]` is the output directory
 *             (default `results`).
 * @return 0 on success.
 */
int main(int argc, char** argv) {
    std::string input_path = argc > 1 ? argv[1] : "examples/input_triangle.txt";
    std::string output_dir = argc > 2 ? argv[2] : "results";
    std::string final_output_path = output_dir + "/results.vtk";

    model truss_model = read_input(input_path);
    int dof_count = 2 * (int)truss_model.nodes.size();

    matrix global_stiffness = assemble(truss_model.nodes, truss_model.elements);
    std::vector<double> full_applied_force = build_f(truss_model.forces, dof_count);

    int load_step_count = truss_model.load_steps > 0 ? truss_model.load_steps : 1;
    int step_number_width = std::max((int)std::to_string(load_step_count).size(), 2);

    std::vector<double> displacements, reaction_forces, stresses;

    std::vector<double> zero_displacement(dof_count, 0.0);
    std::vector<double> zero_stress(truss_model.elements.size(), 0.0);
    std::ostringstream step0_filename;
    step0_filename << output_dir << "/results_step_" << std::setfill('0') << std::setw(step_number_width)
                   << 0 << ".vtk";
    write_vtk(step0_filename.str(), truss_model.nodes, truss_model.elements, zero_displacement,
              zero_stress);
    // clang-format off
    // std::cout << "load step 0/" << load_step_count << " (factor=0, initial conditions): wrote " << step0_filename.str() << "\n";
    // clang-format on

    for (int load_step_index = 1; load_step_index <= load_step_count; ++load_step_index) {
        double load_factor = (double)load_step_index / load_step_count;

        std::vector<double> scaled_force(dof_count);
        for (int dof_index = 0; dof_index < dof_count; ++dof_index)
            scaled_force[dof_index] = full_applied_force[dof_index] * load_factor;

        matrix stiffness_with_bcs = global_stiffness;
        std::vector<double> force_with_bcs = scaled_force;
        apply_bc(stiffness_with_bcs, force_with_bcs, truss_model.boundary_conditions);
        displacements = gauss_solve(stiffness_with_bcs, force_with_bcs);

        reaction_forces = reactions(global_stiffness, displacements, scaled_force);
        stresses = elem_stress(truss_model.nodes, truss_model.elements, displacements);

        std::ostringstream step_filename;
        step_filename << output_dir << "/results_step_" << std::setfill('0')
                      << std::setw(step_number_width) << load_step_index << ".vtk";
        write_vtk(step_filename.str(), truss_model.nodes, truss_model.elements, displacements, stresses);

        // clang-format off
        // std::cout << "load step " << load_step_index << "/" << load_step_count << " (factor=" << load_factor << "):\n";
        // std::cout << "  displacements:\n";
        // for (int i = 0; i < (int)truss_model.nodes.size(); ++i) {
        //     std::cout << "    node " << (i+1) << ": ux=" << displacements[2*i] << " uy=" << displacements[2*i+1] << "\n";
        // }
        // std::cout << "  reactions:\n";
        // for (const auto &boundary_condition : truss_model.boundary_conditions) {
        //     int dof_index = 2*(boundary_condition.node_id-1) + (boundary_condition.dof_index-1);
        //     std::cout << "    node " << boundary_condition.node_id << " dof " << boundary_condition.dof_index << ": " << reaction_forces[dof_index] << "\n";
        // }
        // std::cout << "  axial stresses:\n";
        // for (int i = 0; i < (int)truss_model.elements.size(); ++i) {
        //     std::cout << "    elem " << (i+1) << ": " << stresses[i] << "\n";
        // }
        // clang-format on
    }

    write_vtk(final_output_path, truss_model.nodes, truss_model.elements, displacements, stresses);
    std::cout << "wrote " << (load_step_count + 1)
              << " load-step file(s) (results_step_*.vtk, incl. step 0) and final " << final_output_path
              << "\n";

    return 0;
}
