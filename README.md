# TAMU_Truss

[![CI](https://github.com/jacob-moore22/TAMU_Truss/actions/workflows/ci.yml/badge.svg)](https://github.com/jacob-moore22/TAMU_Truss/actions/workflows/ci.yml)

Basic 2D truss FEM solver (direct stiffness method), code split across `src/`/`include/`. No external deps, hand-rolled Gaussian elimination. Ramps the applied loads from 0 to full value over N load steps and dumps a VTK file per step.

## Build

```
make
```

## Run

```
./truss_solver [input_file] [output_dir]
```

Defaults: `input.txt`, `results/`. Example: `./truss_solver examples/fink_truss.txt results_fink`

## Input format

Plain text, sections: `*NODES`, `*ELEMENTS`, `*BOUNDARIES`, `*FORCES`, `*LOAD_STEPS`. See `input.txt` for the format, `examples/fink_truss.txt` for a second example.

## Output

`results_step_NN.vtk` per load step plus a final `results.vtk`, written to the output dir. Legacy VTK ASCII, load in ParaView. Use `WarpByVector` on `Displacement`, color by `Axial_Stress`.

## Testing

Unit tests use GoogleTest, fetched automatically by CMake — no system install needed.

```
make test
```

or directly via CMake/CTest:

```
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Tests run automatically on every pull request via GitHub Actions (`.github/workflows/ci.yml`).
