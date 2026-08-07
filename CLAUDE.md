# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A basic 2D truss FEM solver using the direct stiffness method, with hand-rolled Gaussian elimination (no external dependencies). It ramps applied loads from 0 to full value over N load steps and writes a legacy ASCII VTK file per step for visualization in ParaView.

## Build

```
make
```

Produces the `truss_solver` binary. `make clean` removes the binary and the default `results/` output dir. There is no test suite.

## Run

```
./truss_solver [input_file] [output_dir]
```

Defaults: `examples/input_triangle.txt`, `results/`. Example: `./truss_solver examples/fink_truss.txt results_fink`

## Architecture

Pipeline in `src/main.cpp`: `read_input` (io.cpp) → `assemble` global stiffness matrix (solver.cpp) → per load step: scale force vector by `load_step_index/load_step_count`, `apply_bc`, `gauss_solve`, compute `reactions` and `elem_stress` → `write_vtk` (vtk_writer.cpp) per step, plus a final `results.vtk`.

- **Data model** (`include/types.h`): `model` holds `nodes`, `elements`, `boundary_conditions`, `forces`, `load_steps`. All node/element IDs in input and internal structs are **1-indexed**; DOF indices into vectors/matrices are 0-indexed as `2*(node_id-1) + (dof_index-1)`, where `dof_index` is 1=X, 2=Y.
- **Solver** (`src/solver.cpp`): `k_local` builds a 4x4 element stiffness matrix from node coords, cross-section area, and Young's modulus; `assemble` scatters element matrices into the global 2N x 2N matrix (`matrix = std::vector<std::vector<double>>`, dense). `apply_bc` enforces boundary conditions via row/column zeroing + unit diagonal (with load vector correction for nonzero prescribed values), not penalty method. `gauss_solve` does Gaussian elimination with partial pivoting on a full dense copy of K per call (no factorization reuse across load steps).
- **I/O format** (`src/io.cpp`): plain-text input with `*NODES`, `*ELEMENTS`, `*BOUNDARIES`, `*FORCES`, `*LOAD_STEPS` sections; `#`-prefixed and blank lines are skipped. See `examples/*.txt` for the format.
- **Output**: legacy VTK ASCII (`vtk_writer.cpp`) with `Displacement` as point vectors and `Axial_Stress` as cell scalars — in ParaView, use `WarpByVector` on `Displacement` and color by `Axial_Stress`.

## Documentation

- All public functions/structs carry Doxygen comments. `make docs` (requires `doxygen`) generates an HTML API reference into `docs/html/` per `Doxyfile`; `make clean-docs` removes it. See `PROCESS_FLOW.md` for a narrative walkthrough of state as it flows through the pipeline.

## Notes for changes

- Load stepping in `main.cpp` re-solves the full system from scratch each step (fresh copy of `global_stiffness`, no incremental/tangent update) — this is a linear-elastic solver, not path-dependent, so that's intentional simplicity, not an oversight to "fix" with caching unless asked.
- Diagnostic print statements per load step in `main.cpp` are deliberately commented out (see `fbe75b3 ENH: Removing excess terminal outputs`); keep that intent if touching that loop.
- Variable/field names were expanded from terse originals (`n1`/`n2`/`a`/`e` → `node1_id`/`node2_id`/`cross_section_area`/`youngs_modulus`, `elems`/`bcs` → `elements`/`boundary_conditions`, `k_global`/`f_vec`/`u_vec` → `stiffness_matrix`/`force_vector`/`displacements`, etc.) for the Doxygen API reference; keep this level of descriptiveness in new code rather than reintroducing single-letter names.
