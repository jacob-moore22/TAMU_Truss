/**
 * @file main.cpp
 * @brief Command-line entry point.
 *
 * Usage: `truss_solver [input_file] [output_dir]`
 *
 * Defaults: `examples/input_triangle.txt` and `results`.
 */
#include <string>

#include "driver.h"
#include "io.h"

/**
 * @brief Reads the input file and runs the analysis.
 *
 * @param argc Number of command-line arguments.
 * @param argv `argv[1]` is the input file and `argv[2]` the output
 *             directory; both are optional.
 * @return 0 on success.
 */
int main(int argc, char** argv) {
    std::string input_path = argc > 1 ? argv[1] : "examples/input_triangle.txt";
    std::string output_dir = argc > 2 ? argv[2] : "results";

    model truss = read_input(input_path);
    run_solver(truss, output_dir);

    return 0;
}
