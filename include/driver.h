#pragma once

#include <string>

#include "types.h"

// Runs the full load-stepping analysis on the model and writes one VTK file
// per load step (results_step_NN.vtk, including step 0) plus a final
// results.vtk into out_dir.
void run_solver(const model& m, const std::string& out_dir);
