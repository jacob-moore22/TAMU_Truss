#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "driver.h"
#include "test_helpers.h"

namespace {

// Parses the displacement vectors (2 per node, z dropped) out of a VTK file
// written by write_vtk.
std::vector<double> read_displacements(const std::string& path) {
    std::vector<std::string> lines = read_lines(path);
    std::vector<double> u;
    auto it =
        std::find(lines.begin(), lines.end(), "VECTORS Displacement float");
    if (it == lines.end()) return u;
    for (++it; it != lines.end() && it->rfind("CELL_DATA", 0) != 0; ++it) {
        std::istringstream ss(*it);
        double x, y, z;
        ss >> x >> y >> z;
        u.push_back(x);
        u.push_back(y);
    }
    return u;
}

std::vector<std::string> sorted_filenames(const fs::path& dir) {
    std::vector<std::string> names;
    for (const auto& e : fs::directory_iterator(dir))
        names.push_back(e.path().filename().string());
    std::sort(names.begin(), names.end());
    return names;
}

}  // namespace

TEST(RunSolver, WritesOneFilePerStepPlusFinal) {
    temp_dir d;
    model m = triangle_model();  // load_steps = 10
    std::string out = d.str() + "/run";

    testing::internal::CaptureStdout();
    run_solver(m, out);
    std::string console = testing::internal::GetCapturedStdout();

    std::vector<std::string> want = {"results.vtk"};
    for (int i = 0; i <= 10; ++i) {
        std::ostringstream n;
        n << "results_step_" << (i < 10 ? "0" : "") << i << ".vtk";
        want.push_back(n.str());
    }
    std::sort(want.begin(), want.end());
    EXPECT_EQ(sorted_filenames(out), want);
    EXPECT_NE(console.find("wrote 11 load-step file(s)"), std::string::npos);
}

TEST(RunSolver, StepZeroIsUndeformed) {
    temp_dir d;
    std::string out = d.str() + "/run";
    testing::internal::CaptureStdout();
    run_solver(triangle_model(), out);
    testing::internal::GetCapturedStdout();

    std::vector<double> u0 = read_displacements(out + "/results_step_00.vtk");
    ASSERT_EQ(u0.size(), 6u);
    for (double v : u0) EXPECT_DOUBLE_EQ(v, 0.0);
}

TEST(RunSolver, FinalFileEqualsLastStep) {
    temp_dir d;
    std::string out = d.str() + "/run";
    testing::internal::CaptureStdout();
    run_solver(triangle_model(), out);
    testing::internal::GetCapturedStdout();

    EXPECT_EQ(read_all(out + "/results.vtk"),
              read_all(out + "/results_step_10.vtk"));
}

TEST(RunSolver, LoadStepsAreLinear) {
    temp_dir d;
    std::string out = d.str() + "/run";
    testing::internal::CaptureStdout();
    run_solver(triangle_model(), out);
    testing::internal::GetCapturedStdout();

    std::vector<double> u5 = read_displacements(out + "/results_step_05.vtk");
    std::vector<double> u10 = read_displacements(out + "/results_step_10.vtk");
    ASSERT_EQ(u5.size(), 6u);
    ASSERT_EQ(u10.size(), 6u);
    for (int i = 0; i < 6; ++i)
        EXPECT_NEAR(u5[i] * 2.0, u10[i], 1e-5 * std::fabs(u10[i]) + 1e-13)
            << "dof " << i;
}

TEST(RunSolver, FinalDisplacementsMatchReference) {
    temp_dir d;
    std::string out = d.str() + "/run";
    testing::internal::CaptureStdout();
    run_solver(triangle_model(), out);
    testing::internal::GetCapturedStdout();

    std::vector<double> u = read_displacements(out + "/results.vtk");
    ASSERT_EQ(u.size(), 6u);
    // VTK output is written with default (6 significant digit) precision.
    for (int i = 0; i < 6; ++i)
        EXPECT_NEAR(u[i], kTriRefU[i], 1e-5 * 5.5e-7) << "dof " << i;
}

TEST(RunSolver, SingleLoadStepUsesTwoDigitPadding) {
    temp_dir d;
    model m = triangle_model();
    m.load_steps = 1;
    std::string out = d.str() + "/run";
    testing::internal::CaptureStdout();
    run_solver(m, out);
    testing::internal::GetCapturedStdout();

    std::vector<std::string> want = {"results.vtk", "results_step_00.vtk",
                                     "results_step_01.vtk"};
    EXPECT_EQ(sorted_filenames(out), want);
}

TEST(RunSolver, ManyLoadStepsWidenPadding) {
    temp_dir d;
    model m = triangle_model();
    m.load_steps = 150;
    std::string out = d.str() + "/run";
    testing::internal::CaptureStdout();
    run_solver(m, out);
    testing::internal::GetCapturedStdout();

    EXPECT_TRUE(fs::exists(out + "/results_step_000.vtk"));
    EXPECT_TRUE(fs::exists(out + "/results_step_007.vtk"));
    EXPECT_TRUE(fs::exists(out + "/results_step_150.vtk"));
    EXPECT_FALSE(fs::exists(out + "/results_step_07.vtk"));
    EXPECT_EQ(sorted_filenames(out).size(), 152u);
}

TEST(RunSolver, ZeroLoadStepsTreatedAsOne) {
    temp_dir d;
    model m = triangle_model();
    m.load_steps = 0;
    std::string out = d.str() + "/run";
    testing::internal::CaptureStdout();
    run_solver(m, out);
    std::string console = testing::internal::GetCapturedStdout();

    std::vector<std::string> want = {"results.vtk", "results_step_00.vtk",
                                     "results_step_01.vtk"};
    EXPECT_EQ(sorted_filenames(out), want);
    EXPECT_NE(console.find("wrote 2 load-step file(s)"), std::string::npos);
}

TEST(RunSolver, UnconstrainedModelThrows) {
    temp_dir d;
    model m = triangle_model();
    m.bcs.clear();
    // Use O(1) stiffness so the singular-pivot check in gauss_solve fires;
    // see GaussSolve.SingularCheckIsAbsolute in test_solver.cpp.
    for (auto& e : m.elems) e.e = 1.0;
    std::string out = d.str() + "/run";
    EXPECT_THROW(run_solver(m, out), std::runtime_error);
}
