#include <string>

#include "driver.h"

int main(int argc, char** argv) {
    std::string in_path = argc > 1 ? argv[1] : "examples/input_triangle.txt";
    std::string out_dir = argc > 2 ? argv[2] : "results";
    return run_analysis(in_path, out_dir);
}
