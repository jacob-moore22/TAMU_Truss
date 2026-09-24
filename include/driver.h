/**
 * @file driver.h
 * @brief Top-level load-stepping analysis.
 */
#pragma once

#include <string>

#include "types.h"

/**
 * @brief Runs the complete analysis on a truss and writes the results.
 *
 * The applied loads are ramped linearly from zero to their full value over
 * `truss.num_load_steps` steps. For each step the truss is solved and a VTK
 * file `results_step_NN.vtk` is written to @p output_dir (step 0 is the
 * undeformed truss). A copy of the final step is also written as
 * `results.vtk`.
 *
 * @param truss      The truss model to analyse.
 * @param output_dir Directory for the VTK files (created if missing).
 * @throws std::runtime_error if the stiffness matrix is singular or an output
 *         file cannot be written.
 */
void run_solver(const model& truss, const std::string& output_dir);
