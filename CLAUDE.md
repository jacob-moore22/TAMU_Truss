# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

2D linear-elastic truss FEM solver (direct stiffness method) in plain C++17. No external dependencies; the linear solve is a hand-rolled dense Gaussian elimination with partial pivoting. Applied loads are ramped linearly from 0 to full value over N load steps, and a legacy-ASCII VTK file is written per step for viewing in ParaView (`WarpByVector` on `Displacement`, color by `Axial_Stress`).

## Commands

```
cmake -S . -B build && cmake --build build -j   # builds ./truss_solver (repo root) + build/tests/truss_tests
ctest --test-dir build --output-on-failure      # run all GoogleTest tests
ctest --test-dir build -R GaussSolve            # run tests matching a regex
./build/tests/truss_tests --gtest_filter='GaussSolve.*'   # same, directly
./truss_solver [input_file] [output_dir]
```

Defaults are `examples/input_triangle.txt` and `results/`. GoogleTest (v1.17.0) is fetched via `FetchContent` at configure time (needs network on first configure); `-DTRUSS_BUILD_TESTS=OFF` skips it.

Tests live in `tests/`, one file per source file (`test_io.cpp`, `test_solver.cpp`, `test_vtk_writer.cpp`, `test_driver.cpp`). `tests/test_helpers.h` provides `TempDirTest` (per-test scratch dir), `example_path()` (resolves `examples/` via the `TRUSS_SOURCE_DIR` compile definition), and VTK read helpers. `SolverIntegration.TriangleMatchesHandCalculation` checks the triangle example against a method-of-joints solution.

## Architecture

`src/main.cpp` only parses argv and calls `run_analysis(in_path, out_dir, out = std::cout)` in `src/driver.cpp`, which drives the pipeline (all sources except `main.cpp` build into the `truss_core` static library that the tests link against):

1. `read_input` (`src/io.cpp`) parses the text input into a `model` (`include/types.h`).
2. `assemble` builds the dense global stiffness matrix once (`k_orig`); `build_f` builds the full load vector.
3. Step 0 is written with zero displacement/stress. Then for each step `1..N`: scale loads by `step/N`, copy `k_orig`, `apply_bc` on the copy, `gauss_solve`, compute `reactions` (against the unmodified `k_orig`) and `elem_stress`, and write `results_step_NN.vtk`.
4. A final `results.vtk` duplicates the last step.

Because the problem is linear, each step is an independent full solve, not an incremental/nonlinear update.

Key conventions:
- **All IDs in input are 1-based**; node `i` maps to global DOFs `2*(i-1)` (x) and `2*(i-1)+1` (y). DOF codes in input are `1=X`, `2=Y`. VTK output uses 0-based point indices.
- Node IDs index directly into `m.nodes` (vector is resized to the max ID), so node IDs must be contiguous; element IDs in the file are ignored (elements are kept in file order).
- Boundary conditions are prescribed displacements (not just zero supports), applied by moving `K[:,dof]*val` to the RHS and zeroing the row/column with a 1 on the diagonal. `apply_bc` and `gauss_solve` mutate/copy their inputs accordingly — keep `k_orig` untouched for reactions.
- `gauss_solve` throws on a near-zero pivot (`< 1e-12`), which usually means an under-constrained structure.
- Units are whatever the input uses; stress is `E * axial strain`.

## Input format

Sections `*NODES` (`id x y`), `*ELEMENTS` (`id n1 n2 area E`), `*BOUNDARIES` (`node dof value`), `*FORCES` (`node dof value`), `*LOAD_STEPS` (`N`). Lines starting with `#` and blank lines are skipped. See `examples/`.

## Formatting

Code is formatted with clang-format using `.clang-format` (Google style, 4-space indent). Run `clang-format -i src/*.cpp include/*.h` after edits; `clang-format --dry-run -Werror src/*.cpp include/*.h` checks without modifying.
