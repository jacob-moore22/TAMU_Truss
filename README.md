# TAMU_Truss

[![CI](https://github.com/jacob-moore22/TAMU_Truss/actions/workflows/ci.yml/badge.svg)](https://github.com/jacob-moore22/TAMU_Truss/actions/workflows/ci.yml) [![Docs](https://github.com/jacob-moore22/TAMU_Truss/actions/workflows/docs.yml/badge.svg)](https://jacob-moore22.github.io/TAMU_Truss/)

Basic 2D truss FEM solver (direct stiffness method), code split across `src/`/`include/`. No external deps, hand-rolled Gaussian elimination. Ramps the applied loads from 0 to full value over N load steps and dumps a VTK file per step.

## Build

```
cmake -S . -B build
cmake --build build -j
```

This builds `./truss_solver` at the repo root and the unit tests in `build/tests/`. GoogleTest is downloaded automatically at configure time; pass `-DTRUSS_BUILD_TESTS=OFF` to skip it.

## Test

```
ctest --test-dir build --output-on-failure
```

## Run

```
./truss_solver [input_file] [output_dir]
```

Defaults: `examples/input_triangle.txt`, `results/`. Example: `./truss_solver examples/fink_truss.txt results_fink`

## Documentation

API docs are generated with Doxygen and published at https://jacob-moore22.github.io/TAMU_Truss/. To build them locally:

```
doxygen Doxyfile        # or: cmake --build build --target docs
```

Output goes to `build/doxygen/html/index.html`.

## Input format

Plain text, sections: `*NODES`, `*ELEMENTS`, `*BOUNDARIES`, `*FORCES`, `*LOAD_STEPS`. See `examples/input_triangle.txt` for the format, `examples/fink_truss.txt` for a second example.

## Output

`results_step_NN.vtk` per load step plus a final `results.vtk`, written to the output dir. Legacy VTK ASCII, load in ParaView. Use `WarpByVector` on `Displacement`, color by `Axial_Stress`.
