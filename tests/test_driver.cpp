#include <gtest/gtest.h>

#include <cmath>
#include <sstream>

#include "driver.h"
#include "test_helpers.h"

class RunAnalysis : public TempDirTest {};

TEST_F(RunAnalysis, WritesOneFilePerStepPlusFinal) {
    std::ostringstream log;
    int rc =
        run_analysis(example_path("input_triangle.txt"), dir_.string(), log);
    EXPECT_EQ(rc, 0);
    for (int step = 0; step <= 10; ++step) {
        std::ostringstream name;
        name << "results_step_" << (step < 10 ? "0" : "") << step << ".vtk";
        EXPECT_TRUE(fs::exists(dir_ / name.str())) << name.str();
    }
    EXPECT_FALSE(fs::exists(dir_ / "results_step_11.vtk"));
    EXPECT_TRUE(fs::exists(dir_ / "results.vtk"));
    EXPECT_NE(log.str().find("wrote 11 load-step file(s)"), std::string::npos);
}

TEST_F(RunAnalysis, StepZeroIsUndeformedAndFinalMatchesLastStep) {
    std::ostringstream log;
    run_analysis(example_path("input_triangle.txt"), dir_.string(), log);

    for (double v : read_vtk_displacements(dir_ / "results_step_00.vtk", 3))
        EXPECT_DOUBLE_EQ(v, 0.0);
    EXPECT_EQ(read_file(dir_ / "results.vtk"),
              read_file(dir_ / "results_step_10.vtk"));
}

TEST_F(RunAnalysis, DisplacementsScaleLinearlyWithLoadStep) {
    std::ostringstream log;
    run_analysis(example_path("input_triangle.txt"), dir_.string(), log);

    auto u_final = read_vtk_displacements(dir_ / "results.vtk", 3);
    auto u_half = read_vtk_displacements(dir_ / "results_step_05.vtk", 3);
    auto u_one = read_vtk_displacements(dir_ / "results_step_01.vtk", 3);
    for (size_t i = 0; i < u_final.size(); ++i) {
        // VTK values are written with 6 significant digits.
        double t = 1e-5 * std::fabs(u_final[i]) + 1e-20;
        EXPECT_NEAR(u_half[i], 0.5 * u_final[i], t);
        EXPECT_NEAR(u_one[i], 0.1 * u_final[i], t);
    }
}

TEST_F(RunAnalysis, PadsStepNumbersToDigitCount) {
    std::ostringstream log;
    run_analysis(example_path("howe_truss.txt"), dir_.string(), log);
    EXPECT_TRUE(fs::exists(dir_ / "results_step_000.vtk"));
    EXPECT_TRUE(fs::exists(dir_ / "results_step_100.vtk"));
    EXPECT_FALSE(fs::exists(dir_ / "results_step_00.vtk"));
}

TEST_F(RunAnalysis, ThrowsOnMissingInput) {
    std::ostringstream log;
    EXPECT_THROW(run_analysis((dir_ / "nope.txt").string(), dir_.string(), log),
                 std::runtime_error);
}
