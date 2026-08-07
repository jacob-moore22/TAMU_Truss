# Graph Report - .  (2026-08-07)

## Corpus Check
- Corpus is ~2,418 words - fits in a single context window. You may not need a graph.

## Summary
- 66 nodes · 113 edges · 7 communities
- Extraction: 87% EXTRACTED · 12% INFERRED · 2% AMBIGUOUS · INFERRED: 13 edges (avg confidence: 0.91)
- Token cost: 0 input · 54,696 output

## Community Hubs (Navigation)
- Type Definitions & Headers
- Docs, Build & Input Format
- FEM Solver Core
- VTK Output
- Element Struct Fields
- Boundary Condition Fields
- Force Struct Fields

## God Nodes (most connected - your core abstractions)
1. `model` - 18 edges
2. `main()` - 11 edges
3. `CLAUDE.md - Repository Guidance` - 10 edges
4. `elem` - 9 edges
5. `README.md - Project Overview` - 9 edges
6. `assemble()` - 8 edges
7. `write_vtk()` - 8 edges
8. `bc` - 6 edges
9. `force` - 6 edges
10. `apply_bc()` - 6 edges

## Surprising Connections (you probably didn't know these)
- `Commit fbe75b3: ENH: Removing excess terminal outputs` --rationale_for--> `main()`  [EXTRACTED]
  CLAUDE.md → src/main.cpp
- `model` --conceptually_related_to--> `1-indexed Node/Element IDs, 0-indexed DOF (2*(node-1)+(dof-1))`  [EXTRACTED]
  include/types.h → CLAUDE.md
- `gauss_solve()` --conceptually_related_to--> `Hand-rolled Gaussian Elimination (partial pivoting, no dependencies)`  [EXTRACTED]
  src/solver.cpp → CLAUDE.md
- `read_input()` --shares_data_with--> `model`  [INFERRED]
  src/io.cpp → include/types.h
- `main()` --shares_data_with--> `model`  [INFERRED]
  src/main.cpp → include/types.h

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Truss Solver Pipeline Data Flow** — include_types_model, src_io_read_input, src_solver_assemble, src_solver_apply_bc, src_solver_gauss_solve, src_vtk_writer_write_vtk, src_main_main [INFERRED 0.85]
- **Truss Input File Format Usage** — claude_io_format, examples_fink_truss, examples_howe_truss, examples_input_triangle, src_io_read_input [INFERRED 0.85]
- **Example Truss Input Suite** — examples_fink_truss, examples_howe_truss, examples_input_triangle [INFERRED 0.80]

## Communities (7 total, 0 thin omitted)

### Community 0 - "Type Definitions & Headers"
Cohesion: 0.16
Nodes (12): array, 1-indexed Node/Element IDs, 0-indexed DOF (2*(node-1)+(dof-1)), vector, model, bcs, elems, forces, load_steps (+4 more)

### Community 1 - "Docs, Build & Input Format"
Cohesion: 0.21
Nodes (15): Build Process (make -> truss_solver binary), Commit fbe75b3: ENH: Removing excess terminal outputs, Direct Stiffness Method (FEM technique), Hand-rolled Gaussian Elimination (partial pivoting, no dependencies), Truss Input File Format (*NODES/*ELEMENTS/*BOUNDARIES/*FORCES/*LOAD_STEPS), Load Stepping (ramp applied loads 0 to full over N steps), CLAUDE.md - Repository Guidance, Fink Truss (classic engineering truss configuration) (+7 more)

### Community 2 - "FEM Solver Core"
Cohesion: 0.36
Nodes (11): matrix, main(), apply_bc(), assemble(), build_f(), node, vector, elem_stress() (+3 more)

### Community 3 - "VTK Output"
Cohesion: 0.25
Nodes (7): ParaView (visualization tool for VTK output), Legacy VTK ASCII Output (Displacement point vectors, Axial_Stress cell scalars), WarpByVector (ParaView filter applied to Displacement), node, string, vector, write_vtk()

### Community 4 - "Element Struct Fields"
Cohesion: 0.40
Nodes (5): elem, a, e, n1, n2

### Community 5 - "Boundary Condition Fields"
Cohesion: 0.50
Nodes (4): bc, dof, node, val

### Community 6 - "Force Struct Fields"
Cohesion: 0.50
Nodes (4): force, dof, node, val

## Ambiguous Edges - Review These
- `CLAUDE.md - Repository Guidance` → `README.md - Project Overview`  [AMBIGUOUS]
  README.md · relation: conceptually_related_to
- `README.md - Project Overview` → `Howe Truss Example Input (examples/howe_truss.txt)`  [AMBIGUOUS]
  README.md · relation: conceptually_related_to

## Knowledge Gaps
- **20 isolated node(s):** `x`, `y`, `n1`, `n2`, `a` (+15 more)
  These have ≤1 connection - possible missing edges or undocumented components.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **What is the exact relationship between `CLAUDE.md - Repository Guidance` and `README.md - Project Overview`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._
- **What is the exact relationship between `README.md - Project Overview` and `Howe Truss Example Input (examples/howe_truss.txt)`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._
- **Why does `model` connect `Type Definitions & Headers` to `Docs, Build & Input Format`, `FEM Solver Core`, `VTK Output`, `Element Struct Fields`, `Boundary Condition Fields`, `Force Struct Fields`?**
  _High betweenness centrality (0.494) - this node is a cross-community bridge._
- **Why does `write_vtk()` connect `VTK Output` to `Type Definitions & Headers`, `FEM Solver Core`, `Element Struct Fields`?**
  _High betweenness centrality (0.220) - this node is a cross-community bridge._
- **Why does `read_input()` connect `Docs, Build & Input Format` to `Type Definitions & Headers`, `FEM Solver Core`?**
  _High betweenness centrality (0.175) - this node is a cross-community bridge._
- **Are the 6 inferred relationships involving `model` (e.g. with `read_input()` and `main()`) actually correct?**
  _`model` has 6 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `main()` (e.g. with `model` and `build_f()`) actually correct?**
  _`main()` has 2 INFERRED edges - model-reasoned connections that need verification._