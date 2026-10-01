#pragma once

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

inline std::string example_path(const std::string& name) {
    return std::string(TRUSS_SOURCE_DIR) + "/examples/" + name;
}

inline void write_file(const fs::path& path, const std::string& text) {
    std::ofstream f(path);
    f << text;
}

inline std::string read_file(const fs::path& path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// Reads the n_nodes (ux, uy) pairs following "VECTORS Displacement float".
inline std::vector<double> read_vtk_displacements(const fs::path& path,
                                                  int n_nodes) {
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line == "VECTORS Displacement float") break;
    }
    std::vector<double> u;
    for (int i = 0; i < n_nodes; ++i) {
        double ux, uy, uz;
        f >> ux >> uy >> uz;
        u.push_back(ux);
        u.push_back(uy);
    }
    return u;
}

// Gives each test its own scratch directory, removed afterwards.
class TempDirTest : public ::testing::Test {
   protected:
    void SetUp() override {
        const auto* info =
            ::testing::UnitTest::GetInstance()->current_test_info();
        dir_ = fs::temp_directory_path() /
               (std::string("truss_test_") + info->test_suite_name() + "_" +
                info->name());
        fs::remove_all(dir_);
        fs::create_directories(dir_);
    }
    void TearDown() override { fs::remove_all(dir_); }

    fs::path dir_;
};
