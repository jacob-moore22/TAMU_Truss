/**
 * @file driver.cpp
 * @brief Implementation of the load-stepped analysis driver.
 */
#include "driver.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

#include "io.h"
#include "solver.h"
#include "vtk_writer.h"

int run_analysis(const std::string& input_path, const std::string& output_dir,
                 std::ostream& log_stream) {
    /// Path of the final-state VTK file.
    std::string final_output_path = output_dir + "/results.vtk";

    /// Truss model read from the input file.
    model truss_model = read_input(input_path);
    /// Total number of global DOFs (2 per node).
    int num_dofs = 2 * (int)truss_model.nodes.size();

    /// Global stiffness matrix without boundary conditions (used for
    /// reactions).
    matrix global_stiffness = assemble(truss_model.nodes, truss_model.elements);
    /// Load vector at the full (final) load level.
    std::vector<double> full_load_vector =
        build_f(truss_model.forces, num_dofs);

    /// Number of load steps (at least 1).
    int num_steps = truss_model.load_steps > 0 ? truss_model.load_steps : 1;
    /// Zero-padded width of step numbers in output file names.
    int step_digit_width = std::max((int)std::to_string(num_steps).size(), 2);

    /// Displacements of the most recent step.
    std::vector<double> displacements;
    /// Reaction forces of the most recent step.
    std::vector<double> reaction_forces;
    /// Element axial stresses of the most recent step.
    std::vector<double> axial_stresses;

    /// Displacements of the undeformed step 0.
    std::vector<double> zero_displacements(num_dofs, 0.0);
    /// Stresses of the unloaded step 0.
    std::vector<double> zero_stresses(truss_model.elements.size(), 0.0);
    /// Builds the step 0 output file path.
    std::ostringstream initial_step_path;
    initial_step_path << output_dir << "/results_step_" << std::setfill('0')
                      << std::setw(step_digit_width) << 0 << ".vtk";
    write_vtk(initial_step_path.str(), truss_model.nodes, truss_model.elements,
              zero_displacements, zero_stresses);
    // std::cout << "load step 0/" << num_steps << " (factor=0, initial
    // conditions): wrote " << initial_step_path.str() << "\n";

    for (int step = 1; step <= num_steps; ++step) {
        /// Fraction of the full load applied at this step.
        double load_factor = (double)step / num_steps;

        /// Load vector scaled to this step.
        std::vector<double> step_load_vector(num_dofs);
        for (int dof_index = 0; dof_index < num_dofs; ++dof_index) {
            step_load_vector[dof_index] =
                full_load_vector[dof_index] * load_factor;
        }

        /// Stiffness matrix with boundary conditions applied.
        matrix constrained_stiffness = global_stiffness;
        /// Load vector with boundary conditions applied.
        std::vector<double> constrained_load_vector = step_load_vector;
        apply_bc(constrained_stiffness, constrained_load_vector,
                 truss_model.boundary_conditions);
        displacements =
            gauss_solve(constrained_stiffness, constrained_load_vector);

        reaction_forces =
            reactions(global_stiffness, displacements, step_load_vector);
        axial_stresses =
            elem_stress(truss_model.nodes, truss_model.elements, displacements);

        /// Builds this step's output file path.
        std::ostringstream step_output_path;
        step_output_path << output_dir << "/results_step_" << std::setfill('0')
                         << std::setw(step_digit_width) << step << ".vtk";
        write_vtk(step_output_path.str(), truss_model.nodes,
                  truss_model.elements, displacements, axial_stresses);

        // std::cout << "load step " << step << "/" << num_steps << " (factor="
        // << load_factor << "):\n";
        // std::cout << "  displacements:\n";
        // for (int i = 0; i < (int)truss_model.nodes.size(); ++i) {
        //     std::cout << "    node " << (i+1) << ": ux=" <<
        //     displacements[2*i]
        //     << " uy=" << displacements[2*i+1] << "\n";
        // }
        // std::cout << "  reactions:\n";
        // for (const auto &constraint : truss_model.boundary_conditions) {
        //     int dof = 2*(constraint.node_id-1) + (constraint.direction-1);
        //     std::cout << "    node " << constraint.node_id << " dof "
        //     << constraint.direction << ": " << reaction_forces[dof] << "\n";
        // }
        // std::cout << "  axial stresses:\n";
        // for (int i = 0; i < (int)truss_model.elements.size(); ++i) {
        //     std::cout << "    elem " << (i+1) << ": " << axial_stresses[i]
        //     << "\n";
        // }
    }

    write_vtk(final_output_path, truss_model.nodes, truss_model.elements,
              displacements, axial_stresses);
    log_stream
        << "wrote " << (num_steps + 1)
        << " load-step file(s) (results_step_*.vtk, incl. step 0) and final "
        << final_output_path << "\n";

    return 0;
}
