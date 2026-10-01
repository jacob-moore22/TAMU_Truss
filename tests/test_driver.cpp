/**
 * @file test_driver.cpp
 * @brief Tests for run_analysis().
 */
#include <gtest/gtest.h>

#include <cmath>
#include <sstream>

#include "driver.h"
#include "test_helpers.h"

/// @brief Fixture for run_analysis() tests; output goes to a scratch dir.
class RunAnalysis : public TempDirTest {};

/// @brief One VTK file is written per step (including step 0) plus the final
///        results file, and a summary is logged.
TEST_F(RunAnalysis, WritesOneFilePerStepPlusFinal) {
    std::ostringstream log_stream;
    int exit_code = run_analysis(example_path("input_triangle.txt"),
                                 scratch_dir_.string(), log_stream);
    EXPECT_EQ(exit_code, 0);
    for (int step = 0; step <= 10; ++step) {
        std::ostringstream file_name;
        file_name << "results_step_" << (step < 10 ? "0" : "") << step
                  << ".vtk";
        EXPECT_TRUE(fs::exists(scratch_dir_ / file_name.str()))
            << file_name.str();
    }
    EXPECT_FALSE(fs::exists(scratch_dir_ / "results_step_11.vtk"));
    EXPECT_TRUE(fs::exists(scratch_dir_ / "results.vtk"));
    EXPECT_NE(log_stream.str().find("wrote 11 load-step file(s)"),
              std::string::npos);
}

/// @brief Step 0 is undeformed and the final file equals the last step.
TEST_F(RunAnalysis, StepZeroIsUndeformedAndFinalMatchesLastStep) {
    std::ostringstream log_stream;
    run_analysis(example_path("input_triangle.txt"), scratch_dir_.string(),
                 log_stream);

    for (double component :
         read_vtk_displacements(scratch_dir_ / "results_step_00.vtk", 3))
        EXPECT_DOUBLE_EQ(component, 0.0);
    EXPECT_EQ(read_file(scratch_dir_ / "results.vtk"),
              read_file(scratch_dir_ / "results_step_10.vtk"));
}

/// @brief Displacements at step k are k/N times the final displacements.
TEST_F(RunAnalysis, DisplacementsScaleLinearlyWithLoadStep) {
    std::ostringstream log_stream;
    run_analysis(example_path("input_triangle.txt"), scratch_dir_.string(),
                 log_stream);

    auto final_displacements =
        read_vtk_displacements(scratch_dir_ / "results.vtk", 3);
    auto half_load_displacements =
        read_vtk_displacements(scratch_dir_ / "results_step_05.vtk", 3);
    auto first_step_displacements =
        read_vtk_displacements(scratch_dir_ / "results_step_01.vtk", 3);
    for (size_t index = 0; index < final_displacements.size(); ++index) {
        // VTK values are written with 6 significant digits.
        double tolerance = 1e-5 * std::fabs(final_displacements[index]) + 1e-20;
        EXPECT_NEAR(half_load_displacements[index],
                    0.5 * final_displacements[index], tolerance);
        EXPECT_NEAR(first_step_displacements[index],
                    0.1 * final_displacements[index], tolerance);
    }
}

/// @brief Step numbers are padded to the digit count of the step total.
TEST_F(RunAnalysis, PadsStepNumbersToDigitCount) {
    std::ostringstream log_stream;
    run_analysis(example_path("howe_truss.txt"), scratch_dir_.string(),
                 log_stream);
    EXPECT_TRUE(fs::exists(scratch_dir_ / "results_step_000.vtk"));
    EXPECT_TRUE(fs::exists(scratch_dir_ / "results_step_100.vtk"));
    EXPECT_FALSE(fs::exists(scratch_dir_ / "results_step_00.vtk"));
}

/// @brief A missing input file throws.
TEST_F(RunAnalysis, ThrowsOnMissingInput) {
    std::ostringstream log_stream;
    EXPECT_THROW(run_analysis((scratch_dir_ / "nope.txt").string(),
                              scratch_dir_.string(), log_stream),
                 std::runtime_error);
}
