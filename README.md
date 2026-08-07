# TAMU_Truss

Basic 2D truss FEM solver (direct stiffness method), code split across `src/`/`include/`. No external deps for the solver itself (hand-rolled Gaussian elimination); the test suite fetches GoogleTest via CMake. Ramps the applied loads from 0 to full value over N load steps and dumps a VTK file per step.

## Build

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

## Test

```
ctest --test-dir build --output-on-failure
```

## Run

```
./build/truss_solver [input_file] [output_dir]
```

Defaults: `examples/input_triangle.txt`, `results/`. Example: `./build/truss_solver examples/fink_truss.txt results_fink`

## Input format

Plain text, sections: `*NODES`, `*ELEMENTS`, `*BOUNDARIES`, `*FORCES`, `*LOAD_STEPS`. See `input.txt` for the format, `examples/fink_truss.txt` for a second example.

## Output

`results_step_NN.vtk` per load step plus a final `results.vtk`, written to the output dir. Legacy VTK ASCII, load in ParaView. Use `WarpByVector` on `Displacement`, color by `Axial_Stress`.
