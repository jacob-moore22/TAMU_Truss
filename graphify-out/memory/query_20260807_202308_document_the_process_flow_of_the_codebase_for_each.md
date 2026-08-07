---
type: "query"
date: "2026-08-07T20:23:08.903398+00:00"
question: "Document the process flow of the codebase for each state variable"
contributor: "graphify"
outcome: "useful"
source_nodes: ["model", "elem", "bc", "force", "matrix", "read_input()", "assemble()", "apply_bc()", "gauss_solve()", "reactions()"]
---

# Q: Document the process flow of the codebase for each state variable

## Answer

Traced domain state via explain on model/elem/bc/force/matrix (EXTRACTED struct-field edges) and the call chain main->read_input->assemble/build_f->apply_bc->gauss_solve->reactions/elem_stress->write_vtk (EXTRACTED calls edges, INFERRED shares_data_with edges to model). Path read_input->write_vtk resolves through model in 2 hops (both INFERRED shares_data_with). Per-load-step locals (k_orig, f_full, k_step, f_step, f_bc, u_vec, r_vec, stresses) are not individually AST-extracted as nodes (they are local vars, not declarations), so their flow was grounded directly against src/main.cpp:10-63 and src/solver.cpp line-by-line, cited by file:line in PROCESS_FLOW.md.

## Outcome

- Signal: useful

## Source Nodes

- model
- elem
- bc
- force
- matrix
- read_input()
- assemble()
- apply_bc()
- gauss_solve()
- reactions()
- elem_stress()
- write_vtk()
- main()