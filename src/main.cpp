/**
 * @file main.cpp
 * @brief Command-line entry point for the truss solver.
 */
#include <string>

#include "driver.h"

/**
 * @brief Run the solver from the command line.
 *
 * Usage: @c truss_solver [input_file] [output_dir]
 *
 * @param argc Number of command-line arguments.
 * @param argv Arguments: optional input file path (default
 *             @c examples/input_triangle.txt) and optional output directory
 *             (default @c results).
 * @return Process exit code from run_analysis().
 */
int main(int argc, char** argv) {
    /// Path to the truss input file.
    std::string input_path = argc > 1 ? argv[1] : "examples/input_triangle.txt";
    /// Directory receiving the VTK output.
    std::string output_dir = argc > 2 ? argv[2] : "results";
    return run_analysis(input_path, output_dir);
}
