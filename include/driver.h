/**
 * @file driver.h
 * @brief Top-level analysis driver: read, solve per load step, write results.
 */
#pragma once

#include <iostream>
#include <string>

/**
 * @brief Run a full load-stepped truss analysis.
 *
 * Reads the model, ramps the loads linearly from 0 to full value over
 * model::load_steps steps, solves each step independently, and writes
 * @c results_step_NN.vtk per step (including the undeformed step 0) plus a
 * final @c results.vtk to @p output_dir. Step numbers are zero-padded to at
 * least two digits.
 *
 * @param input_path Path to the truss input file.
 * @param output_dir Directory for the VTK output (created if missing).
 * @param log_stream Stream that receives the one-line summary.
 * @return 0 on success.
 * @throws std::runtime_error If reading, solving or writing fails.
 */
int run_analysis(const std::string& input_path, const std::string& output_dir,
                 std::ostream& log_stream = std::cout);
