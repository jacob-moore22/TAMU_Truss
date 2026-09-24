#pragma once

#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "types.h"

namespace fs = std::filesystem;

// Unique scratch directory under the system temp dir, removed on destruction.
class temp_dir {
 public:
    temp_dir() {
        static std::atomic<int> counter{0};
        path_ = fs::temp_directory_path() /
                ("truss_test_" + std::to_string(::getpid()) + "_" +
                 std::to_string(counter++));
        fs::create_directories(path_);
    }
    ~temp_dir() {
        std::error_code ec;
        fs::remove_all(path_, ec);
    }
    temp_dir(const temp_dir&) = delete;
    temp_dir& operator=(const temp_dir&) = delete;

    const fs::path& path() const { return path_; }
    std::string str() const { return path_.string(); }

    // Writes contents to <dir>/<name> and returns the full path.
    std::string write_file(const std::string& name,
                           const std::string& contents) const {
        fs::path p = path_ / name;
        std::ofstream f(p);
        f << contents;
        return p.string();
    }

 private:
    fs::path path_;
};

inline std::vector<std::string> read_lines(const std::string& path) {
    std::ifstream f(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(f, line)) lines.push_back(line);
    return lines;
}

inline std::string read_all(const std::string& path) {
    std::ifstream f(path);
    return std::string((std::istreambuf_iterator<char>(f)),
                       std::istreambuf_iterator<char>());
}

// The 3-node triangle from examples/input_triangle.txt, built in code so the
// solver tests do not depend on the parser.
//
//   node 1 (0,0) pinned, node 2 (10,0) roller (y fixed), node 3 (5,10) loaded
//   with Fx=5000, Fy=-10000. A=1.5, E=200e9 for all members.
//
// Statically determinate; reference solution from hand equilibrium:
//   reactions: R1x=-5000, R1y=0, R2y=10000
//   member forces: F12=5000 (T), F23=-11180.34 (C), F13=0
//   stresses = F/A: 3333.33, -7453.56, 0
inline model triangle_model() {
    model m;
    m.nodes = {{0.0, 0.0}, {10.0, 0.0}, {5.0, 10.0}};
    m.elems = {{1, 2, 1.5, 200e9}, {2, 3, 1.5, 200e9}, {1, 3, 1.5, 200e9}};
    m.bcs = {{1, 1, 0.0}, {1, 2, 0.0}, {2, 2, 0.0}};
    m.forces = {{3, 1, 5000.0}, {3, 2, -10000.0}};
    m.load_steps = 10;
    return m;
}

constexpr double kTriRefStress[3] = {3333.3333333333335, -7453.559924999299,
                                     0.0};
constexpr double kTriRefU[6] = {
    0.0, 0.0, 1.6666666666666667e-07, 0.0, 5.4918082904e-07, -2.7459041452e-07};
