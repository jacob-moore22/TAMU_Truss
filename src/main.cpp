#include <string>

#include "driver.h"
#include "io.h"

int main(int argc, char** argv) {
    std::string in_path = argc > 1 ? argv[1] : "examples/input_triangle.txt";
    std::string out_dir = argc > 2 ? argv[2] : "results";

    model m = read_input(in_path);
    run_solver(m, out_dir);

    return 0;
}
