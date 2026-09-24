# TAMU_Truss

Basic 2D truss FEM solver (direct stiffness method), code split across `src/`/`include/`. No external deps, hand-rolled Gaussian elimination. Ramps the applied loads from 0 to full value over N load steps and dumps a VTK file per step.

## Build

```
cmake -B build
cmake --build build
```

Produces `build/truss_solver`. Release (`-O2`) is the default build type.

## Run

```
./build/truss_solver [input_file] [output_dir]
```

Defaults: `examples/input_triangle.txt`, `results/`. Example: `./build/truss_solver examples/fink_truss.txt results_fink`

## Tests

Unit tests use [GoogleTest](https://github.com/google/googletest), fetched by CMake only when `TRUSS_BUILD_TESTS` is on (needs network on first configure):

```
cmake -B build-test -DTRUSS_BUILD_TESTS=ON
cmake --build build-test
ctest --test-dir build-test --output-on-failure
```

Tests live in `tests/`, one file per component (`test_solver.cpp`, `test_io.cpp`, `test_vtk_writer.cpp`, `test_driver.cpp`, `test_types.cpp`).

## Input format

Plain text, sections: `*NODES`, `*ELEMENTS`, `*BOUNDARIES`, `*FORCES`, `*LOAD_STEPS`. See `input.txt` for the format, `examples/fink_truss.txt` for a second example.

## Output

`results_step_NN.vtk` per load step plus a final `results.vtk`, written to the output dir. Legacy VTK ASCII, load in ParaView. Use `WarpByVector` on `Displacement`, color by `Axial_Stress`.
