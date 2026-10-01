/**
 * @file test_helpers.h
 * @brief Shared utilities for the GoogleTest suites.
 */
#pragma once

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

/**
 * @brief Absolute path to a file in the repository's examples/ directory.
 * @param name File name inside examples/.
 * @return Absolute path, independent of the test's working directory.
 */
inline std::string example_path(const std::string& name) {
    return std::string(TRUSS_SOURCE_DIR) + "/examples/" + name;
}

/**
 * @brief Write @p text to @p path, replacing any existing file.
 * @param path Destination file.
 * @param text Content to write.
 */
inline void write_file(const fs::path& path, const std::string& text) {
    /// Output stream for the file.
    std::ofstream file_stream(path);
    file_stream << text;
}

/**
 * @brief Read an entire file into a string.
 * @param path File to read.
 * @return File contents (empty if the file cannot be read).
 */
inline std::string read_file(const fs::path& path) {
    /// Input stream for the file.
    std::ifstream file_stream(path);
    /// Accumulates the file contents.
    std::stringstream buffer;
    buffer << file_stream.rdbuf();
    return buffer.str();
}

/**
 * @brief Read the displacement vectors from a VTK file written by write_vtk().
 * @param path      VTK file to read.
 * @param num_nodes Number of points in the file.
 * @return Flattened (ux, uy) pairs, matching the global DOF layout.
 */
inline std::vector<double> read_vtk_displacements(const fs::path& path,
                                                  int num_nodes) {
    /// Input stream for the VTK file.
    std::ifstream file_stream(path);
    /// Current line while searching for the displacement header.
    std::string line;
    while (std::getline(file_stream, line)) {
        if (line == "VECTORS Displacement float") break;
    }
    /// Flattened displacements read so far.
    std::vector<double> displacements;
    for (int node_index = 0; node_index < num_nodes; ++node_index) {
        double disp_x, disp_y, disp_z;
        file_stream >> disp_x >> disp_y >> disp_z;
        displacements.push_back(disp_x);
        displacements.push_back(disp_y);
    }
    return displacements;
}

/// @brief Test fixture giving each test its own scratch directory, removed
///        after the test.
class TempDirTest : public ::testing::Test {
   protected:
    /// @brief Create an empty scratch directory named after the test.
    void SetUp() override {
        /// Metadata of the currently running test.
        const auto* test_info =
            ::testing::UnitTest::GetInstance()->current_test_info();
        scratch_dir_ = fs::temp_directory_path() /
                       (std::string("truss_test_") +
                        test_info->test_suite_name() + "_" + test_info->name());
        fs::remove_all(scratch_dir_);
        fs::create_directories(scratch_dir_);
    }

    /// @brief Delete the scratch directory and its contents.
    void TearDown() override { fs::remove_all(scratch_dir_); }

    fs::path scratch_dir_;  ///< Per-test temporary directory.
};
