# Process Flow

How data moves through the truss solver, variable by variable. Written for new contributors who need to know *what* holds the state at each stage of the pipeline and *which function* mutates it next. Generated with the aid of a [graphify](https://github.com/safishamsi/graphify) knowledge graph built from this repo (`graphify-out/graph.json`, `graphify-out/GRAPH_REPORT.md`) — struct/function relationships below are graph-verified (EXTRACTED = read directly from source, INFERRED = graph-reasoned); the per-load-step local variables are cited directly by `file:line` since they're locals, not declarations, and AST extraction doesn't produce graph nodes for those.

> Every function referenced below carries a Doxygen comment in its header (`include/*.h`); run `make docs` to generate a browsable HTML API reference into `docs/html/`. Variable and struct-field names here match the current source — they were expanded from terse originals (`n1`/`n2`/`a`/`e`, `k_global`/`f_vec`/`u_vec`, etc.) specifically so this document and the generated API docs stay self-explanatory.

## 1. Pipeline overview

```
read_input()          assemble()             build_f()
     |                     |                      |
     v                     v                      v
   model  ---> global_stiffness (matrix)   full_applied_force (vector)
     |                     |                      |
     |                     |    (copied per load step, load_step_index = 1..load_step_count)
     |                     v                      v
     |          stiffness_with_bcs  <--apply_bc--  scaled_force --> force_with_bcs
     |                     |                                          |
     |                     +----------------> gauss_solve() <---------+
     |                                              |
     |                                              v
     |                                       displacements ---> elem_stress() -> stresses
     |                                              |
     |                     reactions(global_stiffness, displacements, scaled_force) -> reaction_forces
     |                                              |
     +---------- write_vtk(truss_model.nodes, truss_model.elements, displacements, stresses) ----> results_step_NN.vtk
```

All of this runs inside `main()` (`src/main.cpp:25`), the only place that owns the state above — `solver.cpp`, `io.cpp`, and `vtk_writer.cpp` are pure functions that take state in and hand new state back, they don't retain anything between calls.

## 2. Domain state — `model` and its structs

This is the persistent state, loaded once and read (never mutated) for the rest of the run. Confirmed via `graphify explain model` — every edge below is EXTRACTED (read directly from `include/types.h`); field names have since been expanded (see note below the table).

| Struct | Fields | Populated by | Read by |
|---|---|---|---|
| `node` (`include/types.h:6`) | `x`, `y` | `read_input()` | `assemble()`, `elem_stress()`, `write_vtk()` |
| `elem` (`include/types.h:12`) | `node1_id`, `node2_id`, `cross_section_area`, `youngs_modulus` | `read_input()` | `assemble()`, `elem_stress()`, `write_vtk()` |
| `bc` (`include/types.h:21`) | `node_id`, `dof_index`, `value` | `read_input()` | `apply_bc()` |
| `force` (`include/types.h:28`) | `node_id`, `dof_index`, `value` | `read_input()` | `build_f()` |
| `model` (`include/types.h:36`) | `nodes`, `elements`, `boundary_conditions`, `forces`, `load_steps` | `read_input()` | `main()`, `assemble()`, `apply_bc()`, `gauss_solve()`*, `write_vtk()` |

\* `gauss_solve()`'s edge to `model` is graph-INFERRED (shared-data reasoning) rather than a direct reference — in the actual source it never touches `model` directly, it only operates on the `matrix`/`vector` handed to it by `main()`. Flagged here rather than silently trusted, per the graph's own confidence tagging.

> **Naming note:** `elem`'s fields were originally `n1`/`n2`/`a`/`e` and `model`'s were `elems`/`bcs`; `bc`/`force`'s were `node`/`dof`/`val`. They were renamed to `node1_id`/`node2_id`/`cross_section_area`/`youngs_modulus`, `elements`/`boundary_conditions`, and `node_id`/`dof_index`/`value` respectively for readability. The `graphify-out/graph.json` knowledge graph still has the old field names cached from the last extraction — re-run `/graphify --update` if you need the graph itself to reflect this rename.

**Flow:** `read_input(input_path)` (`src/io.cpp:18`) parses the `*NODES/*ELEMENTS/*BOUNDARIES/*FORCES/*LOAD_STEPS` sections of the input file into one `model truss_model`, 1-indexed on node/element IDs. `main()` (`src/main.cpp:30`) holds `truss_model` for the rest of the run and passes its fields — never the whole struct — into every downstream function.

## 3. Setup state — built once, reused every load step

| Variable | Type | Created | Line | Consumed by |
|---|---|---|---|---|
| `dof_count` | `int` | `2 * truss_model.nodes.size()` | `main.cpp:31` | sizes every force/displacement vector below |
| `global_stiffness` | `matrix` (`vector<vector<double>>`) | `assemble(truss_model.nodes, truss_model.elements)` | `main.cpp:33` | copied into `stiffness_with_bcs` each step; also passed untouched into `reactions()` |
| `full_applied_force` | `vector<double>` | `build_f(truss_model.forces, dof_count)` | `main.cpp:34-35` | scaled into `scaled_force` each step |
| `load_step_count` | `int` | `truss_model.load_steps > 0 ? truss_model.load_steps : 1` | `main.cpp:37-38` | loop bound |

`assemble()` (`src/solver.cpp:54`) builds the global stiffness matrix by calling `k_local()` per element and scattering each element's 4x4 local stiffness matrix into the `2*n_nodes` x `2*n_nodes` global one, keyed off each element's DOF indices (`2*(node_id-1) + (dof_index-1)`, 1=X / 2=Y as noted in `CLAUDE.md`). `global_stiffness` is never mutated after this call — it's the pristine matrix that BCs get re-applied to fresh on every load step.

`build_f(truss_model.forces, dof_count)` (`src/solver.cpp:84`) scatters every `force` entry into a flat `dof_count`-length vector. `full_applied_force` represents the *full* applied load — the load-stepping loop scales a fresh copy of it down, it never edits `full_applied_force` itself.

## 4. Per-load-step state — rebuilt from scratch every iteration

`main.cpp:55-98` loops `load_step_index = 1..load_step_count`. Every variable in this section is **local to one loop iteration** — nothing here persists into the next step except by being copied fresh from the step-0 state above. This is deliberate: the solver is linear-elastic and not path-dependent, so there's no reason to reuse a prior step's stiffness matrix or factorization (see `CLAUDE.md`'s note on this).

| Variable | Type | Created from | Transform | Line |
|---|---|---|---|---|
| `load_factor` | `double` | `load_step_index / load_step_count` | ramps 0→1 across the loop | `main.cpp:57` |
| `scaled_force` | `vector<double>` | `full_applied_force[i] * load_factor` | scaled load for this step | `main.cpp:59-62` |
| `stiffness_with_bcs` | `matrix` | copy of `global_stiffness` | boundary rows/cols zeroed by `apply_bc()` | `main.cpp:64` |
| `force_with_bcs` | `vector<double>` | copy of `scaled_force` | corrected in place by `apply_bc()` for nonzero prescribed BCs | `main.cpp:65` |
| `displacements` | `vector<double>` | `gauss_solve(stiffness_with_bcs, force_with_bcs)` | solved displacement | `main.cpp:68` |
| `reaction_forces` | `vector<double>` | `reactions(global_stiffness, displacements, scaled_force)` | `K·u - f`, evaluated against the **unmodified** `global_stiffness` | `main.cpp:70-71` |
| `stresses` | `vector<double>` | `elem_stress(truss_model.nodes, truss_model.elements, displacements)` | axial stress per element from `displacements` | `main.cpp:72-73` |

Step-by-step:

1. **`apply_bc(stiffness_with_bcs, force_with_bcs, truss_model.boundary_conditions)`** (`src/solver.cpp:108`) — for every `bc`, it moves the prescribed value's contribution out of the RHS (`force_vector[row] -= stiffness_matrix[row][dof_index]*value` for every row), then zeroes that DOF's row/column in `stiffness_with_bcs` and sets the diagonal to `1.0` and `force_with_bcs[dof_index] = value`. This is a row/column elimination, not a penalty method — `stiffness_with_bcs` is genuinely mutated in place.
2. **`gauss_solve(stiffness_with_bcs, force_with_bcs)`** (`src/solver.cpp:137`) — takes the two arguments **by value** (note the signature has no `&`), so it's free to do in-place partial-pivoted Gaussian elimination without touching the caller's `stiffness_with_bcs`/`force_with_bcs`, then back-substitutes into a new `displacements` vector.
3. **`reactions(global_stiffness, displacements, scaled_force)`** (`src/solver.cpp:191`) — computed against `global_stiffness`, the *original* unconstrained stiffness matrix, not `stiffness_with_bcs`. This is what makes it read as a genuine reaction force at the constrained DOFs rather than just re-deriving the identity row `apply_bc` wrote.
4. **`elem_stress(truss_model.nodes, truss_model.elements, displacements)`** (`src/solver.cpp:216`) — recomputes each element's direction cosines from `truss_model.nodes` (not stored anywhere else) and combines with `displacements` at that element's 4 DOFs to get axial strain, then stress = `youngs_modulus * strain`.
5. **`write_vtk(...)`** (`src/vtk_writer.cpp:22`) is called with `displacements` and `stresses` at the end of every iteration — one `results_step_NN.vtk` per step, plus a step-0 file written before the loop with all-zero `zero_displacement`/`zero_stress` (`main.cpp:44-50`), and a final `results.vtk` written once after the loop with the *last* step's `displacements`/`stresses` (`main.cpp:100-101`).

## 5. Where state exits the program

`write_vtk()` is the only sink. It never mutates its inputs — it reads `truss_model.nodes`/`truss_model.elements` for geometry and connectivity, and `displacements`/`stresses` for the per-step results, and serializes all of it into legacy ASCII VTK (`POINTS`, `CELLS`, `Displacement` as point vectors, `Axial_Stress` as cell scalars). Nothing computed in this pipeline is retained past the call that produces it except through the file system.

## 6. Querying this further

The graph backing this document is checked into `graphify-out/`. To explore live:

```bash
graphify explain "model"        # everything connected to the domain struct
graphify path "read_input" "write_vtk"   # how domain state reaches the output sink
graphify query "load step displacement stress"
```

See `GRAPH_REPORT.md` in the same directory for community structure, god nodes, and open AMBIGUOUS edges (e.g. the README vs. CLAUDE.md default-input-file discrepancy, still unresolved as of this writing). Note the graph itself was last built before the variable/field rename described in §2 — its node labels for `elem`/`bc`/`force` fields are stale until the next `/graphify --update`.

For generated API documentation (every function/struct above, cross-linked), run `make docs` and open `docs/html/index.html`.
