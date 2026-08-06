# Graph Report - .  (2026-08-05)

## Corpus Check
- Corpus is ~2,048 words - fits in a single context window. You may not need a graph.

## Summary
- 60 nodes · 91 edges · 8 communities (6 shown, 2 thin omitted)
- Extraction: 86% EXTRACTED · 14% INFERRED · 0% AMBIGUOUS · INFERRED: 13 edges (avg confidence: 0.82)
- Token cost: 0 input · 34,279 output

## Community Hubs (Navigation)
- Model & Node Data Structures
- FEM Solver Core
- Documentation & Example Models
- Element Struct & VTK Output
- Boundary Condition Struct
- Force Struct
- Input File Parser
- Project Overview

## God Nodes (most connected - your core abstractions)
1. `model` - 12 edges
2. `elem` - 9 edges
3. `main()` - 9 edges
4. `assemble()` - 7 edges
5. `bc` - 6 edges
6. `force` - 6 edges
7. `write_vtk()` - 6 edges
8. `truss_solver (CLI executable)` - 6 edges
9. `apply_bc()` - 5 edges
10. `elem_stress()` - 5 edges

## Surprising Connections (you probably didn't know these)
- `assemble()` --references--> `elem`  [EXTRACTED]
  src/solver.cpp → include/types.h
- `elem_stress()` --references--> `elem`  [EXTRACTED]
  src/solver.cpp → include/types.h
- `apply_bc()` --references--> `bc`  [EXTRACTED]
  src/solver.cpp → include/types.h
- `build_f()` --references--> `force`  [EXTRACTED]
  src/solver.cpp → include/types.h
- `read_input()` --references--> `model`  [EXTRACTED]
  src/io.cpp → include/types.h

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Truss Input File Format used by example models** — readme_input_format, examples_fink_truss_model, examples_howe_truss_model, examples_input_triangle_model [INFERRED 0.85]
- **Example Truss Models Suite** — examples_fink_truss_model, examples_howe_truss_model, examples_input_triangle_model [INFERRED 0.85]

## Communities (8 total, 2 thin omitted)

### Community 0 - "Model & Node Data Structures"
Cohesion: 0.17
Nodes (11): array, vector, model, bcs, elems, forces, load_steps, nodes (+3 more)

### Community 1 - "FEM Solver Core"
Cohesion: 0.36
Nodes (11): matrix, main(), apply_bc(), assemble(), build_f(), node, vector, elem_stress() (+3 more)

### Community 2 - "Documentation & Example Models"
Cohesion: 0.31
Nodes (10): Fink Truss Example Model, Howe Truss Example Model, Triangle Truss Example Model, Direct Stiffness Method (FEM approach), Hand-rolled Gaussian Elimination, Truss Input File Format (*NODES/*ELEMENTS/*BOUNDARIES/*FORCES/*LOAD_STEPS), Load Ramping over N Load Steps, ParaView (+2 more)

### Community 3 - "Element Struct & VTK Output"
Cohesion: 0.20
Nodes (9): elem, a, e, n1, n2, node, string, vector (+1 more)

### Community 4 - "Boundary Condition Struct"
Cohesion: 0.50
Nodes (4): bc, dof, node, val

### Community 5 - "Force Struct"
Cohesion: 0.50
Nodes (4): force, dof, node, val

## Knowledge Gaps
- **19 isolated node(s):** `x`, `y`, `n1`, `n2`, `a` (+14 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `model` connect `Model & Node Data Structures` to `Element Struct & VTK Output`, `Boundary Condition Struct`, `Force Struct`, `Input File Parser`?**
  _High betweenness centrality (0.241) - this node is a cross-community bridge._
- **Why does `elem` connect `Element Struct & VTK Output` to `Model & Node Data Structures`, `FEM Solver Core`?**
  _High betweenness centrality (0.223) - this node is a cross-community bridge._
- **Why does `main()` connect `FEM Solver Core` to `Element Struct & VTK Output`, `Input File Parser`?**
  _High betweenness centrality (0.117) - this node is a cross-community bridge._
- **Are the 8 inferred relationships involving `main()` (e.g. with `read_input()` and `apply_bc()`) actually correct?**
  _`main()` has 8 INFERRED edges - model-reasoned connections that need verification._
- **What connects `x`, `y`, `n1` to the rest of the system?**
  _19 weakly-connected nodes found - possible documentation gaps or missing edges._