# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A basic 2D truss FEM solver using the direct stiffness method, with hand-rolled Gaussian elimination (no external dependencies). It ramps applied loads from 0 to full value over N load steps and writes a legacy ASCII VTK file per step for visualization in ParaView.

## Build

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

Produces `build/truss_solver`. First configure fetches GoogleTest via `FetchContent` (network required once; cached under `build/_deps` after). `rm -rf build` is the equivalent of the old `make clean`.

## Run

```
./build/truss_solver [input_file] [output_dir]
```

Defaults: `examples/input_triangle.txt`, `results/`. Example: `./build/truss_solver examples/fink_truss.txt results_fink`

## Test

```
ctest --test-dir build --output-on-failure
```

GoogleTest suite in `tests/` (`test_solver.cpp`, `test_vtk_writer.cpp`, `test_integration.cpp`) covers everything in `src/solver.cpp` and `src/vtk_writer.cpp`, plus one full-pipeline integration test against a hand-built model (independently cross-checked, not derived from this code). `src/io.cpp`/`include/io.h` (`read_input`) and `src/main.cpp` are intentionally untested — `read_input` is thin text parsing, `main` is the CLI entry point. Runs automatically on every PR via `.github/workflows/tests.yml` (also triggerable manually via `workflow_dispatch`).

## Architecture

Pipeline in `src/main.cpp`: `read_input` (io.cpp) → `assemble` global stiffness matrix (solver.cpp) → per load step: scale force vector by `load_step_index/load_step_count`, `apply_bc`, `gauss_solve`, compute `reactions` and `elem_stress` → `write_vtk` (vtk_writer.cpp) per step, plus a final `results.vtk`.

- **Data model** (`include/types.h`): `model` holds `nodes`, `elements`, `boundary_conditions`, `forces`, `load_steps`. All node/element IDs in input and internal structs are **1-indexed**; DOF indices into vectors/matrices are 0-indexed as `2*(node_id-1) + (dof_index-1)`, where `dof_index` is 1=X, 2=Y.
- **Solver** (`src/solver.cpp`): `k_local` builds a 4x4 element stiffness matrix from node coords, cross-section area, and Young's modulus; `assemble` scatters element matrices into the global 2N x 2N matrix (`matrix = std::vector<std::vector<double>>`, dense). `apply_bc` enforces boundary conditions via row/column zeroing + unit diagonal (with load vector correction for nonzero prescribed values), not penalty method. `gauss_solve` does Gaussian elimination with partial pivoting on a full dense copy of K per call (no factorization reuse across load steps).
- **I/O format** (`src/io.cpp`): plain-text input with `*NODES`, `*ELEMENTS`, `*BOUNDARIES`, `*FORCES`, `*LOAD_STEPS` sections; `#`-prefixed and blank lines are skipped. See `examples/*.txt` for the format.
- **Output**: legacy VTK ASCII (`vtk_writer.cpp`) with `Displacement` as point vectors and `Axial_Stress` as cell scalars — in ParaView, use `WarpByVector` on `Displacement` and color by `Axial_Stress`.

## Documentation

- All public functions/structs carry Doxygen comments. `cmake --build build --target docs` (requires `doxygen` + `graphviz`) generates an HTML API reference into `docs/html/` per `Doxyfile`; `rm -rf docs` removes it. `cmake --build build --target format` runs `clang-format` over `src/`, `include/`, and `tests/`. See `PROCESS_FLOW.md` for a narrative walkthrough of state as it flows through the pipeline.
- Graphviz diagrams (`HAVE_DOT`/`COLLABORATION_GRAPH`/`CALL_GRAPH`/`CALLER_GRAPH` in `Doxyfile`) require the `graphviz` package (provides `dot`) in addition to `doxygen` — the CI workflow installs both. There's no separate top-level "graph" page: diagrams are embedded per-entity. Struct composition (e.g. `model` → `node`/`elem`/`bc`/`force`) shows as a "Collaboration diagram" on each struct's page under **Data Structures**; function call relationships show as "Call graph"/"Caller graph" inline in each function's docs under **Files** (e.g. `src/solver.cpp`).
- `.github/workflows/docs.yml` builds and deploys the Doxygen HTML to GitHub Pages on every PR, on push to `main`, and on manual `workflow_dispatch`. It uses the official `actions/deploy-pages` flow, which serves **one site at a time** — a PR's build makes that branch's docs live at the Pages URL (satisfying "viewable before merge"), and pushing to `main` (e.g. after a merge) redeploys `main`'s docs to reclaim it. Two PRs open simultaneously will clobber each other's preview; there's no per-branch subdirectory isolation. Requires GitHub Pages enabled once via repo Settings → Pages → Source → "GitHub Actions" (a one-time manual step, not something this workflow or repo config can do on its own).

## Notes for changes

- Load stepping in `main.cpp` re-solves the full system from scratch each step (fresh copy of `global_stiffness`, no incremental/tangent update) — this is a linear-elastic solver, not path-dependent, so that's intentional simplicity, not an oversight to "fix" with caching unless asked.
- Diagnostic print statements per load step in `main.cpp` are deliberately commented out (see `fbe75b3 ENH: Removing excess terminal outputs`); keep that intent if touching that loop.
- Variable/field names were expanded from terse originals (`n1`/`n2`/`a`/`e` → `node1_id`/`node2_id`/`cross_section_area`/`youngs_modulus`, `elems`/`bcs` → `elements`/`boundary_conditions`, `k_global`/`f_vec`/`u_vec` → `stiffness_matrix`/`force_vector`/`displacements`, etc.) for the Doxygen API reference; keep this level of descriptiveness in new code rather than reintroducing single-letter names.
