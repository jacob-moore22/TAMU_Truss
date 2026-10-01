# Variable rename map

Record of every variable name in the codebase as it was before the rename, and what it became. Scope is variables only: struct fields, function parameters, locals, loop indices, test fixtures and constants. Type names (`node`, `elem`, `bc`, `force`, `model`, `matrix`) and function names are unchanged.

"(kept)" means the original name was already descriptive or is a fixed convention (e.g. `argc`/`argv`, coordinate `x`/`y`).

## include/types.h

| Struct | Old | New | Meaning |
|---|---|---|---|
| `node` | `x` | `x` (kept) | x coordinate |
| `node` | `y` | `y` (kept) | y coordinate |
| `elem` | `n1` | `start_node` | 1-based ID of first end node |
| `elem` | `n2` | `end_node` | 1-based ID of second end node |
| `elem` | `a` | `area` | cross-sectional area |
| `elem` | `e` | `youngs_modulus` | Young's modulus |
| `bc` | `node` | `node_id` | 1-based node ID |
| `bc` | `dof` | `direction` | 1 = x, 2 = y |
| `bc` | `val` | `displacement` | prescribed displacement |
| `force` | `node` | `node_id` | 1-based node ID |
| `force` | `dof` | `direction` | 1 = x, 2 = y |
| `force` | `val` | `magnitude` | applied force |
| `model` | `nodes` | `nodes` (kept) | node list |
| `model` | `elems` | `elements` | element list |
| `model` | `bcs` | `boundary_conditions` | prescribed displacements |
| `model` | `forces` | `forces` (kept) | applied point loads |
| `model` | `load_steps` | `load_steps` (kept) | number of load increments |

## include/io.h, src/io.cpp — `read_input`

| Old | New | Kind |
|---|---|---|
| `path` | `input_path` | parameter |
| `f` | `input_file` | local |
| `m` | `truss_model` | local |
| `line` | `line` (kept) | local |
| `section` | `current_section` | local |
| `ss` | `line_stream` | local |
| `first` | `first_token` | local |
| `data` | `field_stream` | local |
| `id` (nodes) | `node_id` | local |
| `x`, `y` | `x_coord`, `y_coord` | locals |
| `id` (elements) | `element_id` | local (read and discarded) |
| `n1`, `n2` | `start_node`, `end_node` | locals |
| `a` | `area` | local |
| `e` | `youngs_modulus` | local |
| `nd` | `node_id` | local (boundaries and forces) |
| `dof` | `direction` | local (boundaries and forces) |
| `val` (boundaries) | `displacement` | local |
| `val` (forces) | `magnitude` | local |

## include/solver.h, src/solver.cpp

### `k_local`
| Old | New | Kind |
|---|---|---|
| `a` | `start_node` | parameter |
| `b` | `end_node` | parameter |
| `area` | `area` (kept) | parameter |
| `e` | `youngs_modulus` | parameter |
| `dx`, `dy` | `delta_x`, `delta_y` | locals |
| `l` | `length` | local |
| `c` | `cos_theta` | local |
| `s` | `sin_theta` | local |
| `k` | `axial_stiffness` | local |
| `ke` | `element_stiffness` | local |

### `assemble`
| Old | New | Kind |
|---|---|---|
| `nodes` | `nodes` (kept) | parameter |
| `elems` | `elements` | parameter |
| `n_dof` | `num_dofs` | local |
| `k_global` | `global_stiffness` | local |
| `el` | `element` | loop variable |
| `ke` | `element_stiffness` | local |
| `dofs` | `global_dofs` | local |
| `i`, `j` | `row`, `col` | loop indices |

### `build_f`
| Old | New | Kind |
|---|---|---|
| `forces` | `forces` (kept) | parameter |
| `n_dof` | `num_dofs` | parameter |
| `f_vec` | `load_vector` | local |
| `fr` | `point_load` | loop variable |
| `dof` | `global_dof` | local |

### `apply_bc`
| Old | New | Kind |
|---|---|---|
| `k_global` | `stiffness` | parameter |
| `f_vec` | `load_vector` | parameter |
| `bcs` | `boundary_conditions` | parameter |
| `n_dof` | `num_dofs` | local |
| `b` | `constraint` | loop variable |
| `dof` | `global_dof` | local |
| `i` | `index` | loop index |

### `gauss_solve`
| Old | New | Kind |
|---|---|---|
| `k_global` | `coefficients` | parameter |
| `f_vec` | `rhs` | parameter |
| `n` | `num_equations` | local |
| `p` | `pivot` | loop index |
| `max_row` | `pivot_row` | local |
| `max_val` | `pivot_magnitude` | local |
| `i`, `j` | `row`, `col` | loop indices |
| `factor` | `elimination_factor` | local |
| `u_vec` | `solution` | local |
| `sum` | `remainder` | local |

### `reactions`
| Old | New | Kind |
|---|---|---|
| `k_global` | `global_stiffness` | parameter |
| `u_vec` | `displacements` | parameter |
| `f_vec` | `applied_loads` | parameter |
| `n` | `num_dofs` | local |
| `r_vec` | `reaction_forces` | local |
| `i`, `j` | `row`, `col` | loop indices |
| `sum` | `internal_force` | local |

### `elem_stress`
| Old | New | Kind |
|---|---|---|
| `nodes` | `nodes` (kept) | parameter |
| `elems` | `elements` | parameter |
| `u_vec` | `displacements` | parameter |
| `stresses` | `axial_stresses` | local |
| `el` | `element` | loop variable |
| `a`, `b` | `start_node`, `end_node` | locals |
| `dx`, `dy` | `delta_x`, `delta_y` | locals |
| `l` | `length` | local |
| `c`, `s` | `cos_theta`, `sin_theta` | locals |
| `dofs` | `global_dofs` | local |
| `ue` | `element_displacements` | local |
| `strain` | `axial_strain` | local |

## include/vtk_writer.h, src/vtk_writer.cpp — `write_vtk`

| Old | New | Kind |
|---|---|---|
| `path` | `output_path` | parameter |
| `nodes` | `nodes` (kept) | parameter |
| `elems` | `elements` | parameter |
| `u_vec` | `displacements` | parameter |
| `stresses` | `axial_stresses` | parameter |
| `p` | `file_path` | local |
| `f` | `vtk_file` | local |
| `n_nodes` | `num_nodes` | local |
| `n_elems` | `num_elements` | local |
| `n` | `point` | loop variable |
| `el` | `element` | loop variable |
| `i` (cell types) | `cell_index` | loop index |
| `i` (displacements) | `node_index` | loop index |
| `s` | `stress` | loop variable |

## include/driver.h, src/driver.cpp — `run_analysis`

| Old | New | Kind |
|---|---|---|
| `in_path` | `input_path` | parameter |
| `out_dir` | `output_dir` | parameter |
| `out` | `log_stream` | parameter |
| `out_path` | `final_output_path` | local |
| `m` | `truss_model` | local |
| `n_dof` | `num_dofs` | local |
| `k_orig` | `global_stiffness` | local |
| `f_full` | `full_load_vector` | local |
| `n_steps` | `num_steps` | local |
| `width` | `step_digit_width` | local |
| `u_vec` | `displacements` | local |
| `r_vec` | `reaction_forces` | local |
| `stresses` | `axial_stresses` | local |
| `u_zero` | `zero_displacements` | local |
| `stress_zero` | `zero_stresses` | local |
| `name0` | `initial_step_path` | local |
| `step` | `step` (kept) | loop index |
| `factor` | `load_factor` | local |
| `f_step` | `step_load_vector` | local |
| `i` | `dof_index` | loop index |
| `k_step` | `constrained_stiffness` | local |
| `f_bc` | `constrained_load_vector` | local |
| `name` | `step_output_path` | local |

## src/main.cpp — `main`

| Old | New | Kind |
|---|---|---|
| `argc`, `argv` | (kept) | parameters |
| `in_path` | `input_path` | local |
| `out_dir` | `output_dir` | local |

## tests/

| File | Old | New |
|---|---|---|
| `test_helpers.h` | `f` (`write_file`/`read_file`/`read_vtk_displacements`) | `file_stream` |
| `test_helpers.h` | `ss` | `buffer` |
| `test_helpers.h` | `n_nodes` | `num_nodes` |
| `test_helpers.h` | `u` | `displacements` |
| `test_helpers.h` | `ux`, `uy`, `uz` | `disp_x`, `disp_y`, `disp_z` |
| `test_helpers.h` | `i` | `node_index` |
| `test_helpers.h` | `info` | `test_info` |
| `test_helpers.h` | `dir_` | `scratch_dir_` |
| `test_io.cpp` | `m`, `fink`, `howe` | `truss_model`, `fink_model`, `howe_model` |
| `test_io.cpp` | `path` | `input_path` |
| `test_solver.cpp` | `kE`, `kA` | `kYoungsModulus`, `kArea` |
| `test_solver.cpp` | `tol(scale)` | `tol(magnitude)` |
| `test_solver.cpp` | `ke` | `element_stiffness` |
| `test_solver.cpp` | `k` (expected scalar) | `expected_stiffness` |
| `test_solver.cpp` | `k` (matrix) | `stiffness` |
| `test_solver.cpp` | `l`, `half` | `length`, `half_stiffness` |
| `test_solver.cpp` | `row` (row sum) | `row_sum` |
| `test_solver.cpp` | `i`, `j` | `row`, `col` |
| `test_solver.cpp` | `k1`, `k2` | `short_bar_stiffness`, `long_bar_stiffness` |
| `test_solver.cpp` | `f` | `loads` |
| `test_solver.cpp` | `u` | `displacements` |
| `test_solver.cpp` | `r` | `reaction_forces` |
| `test_solver.cpp` | `s`, `s1`, `s2` | `axial_stresses`, `translation_stresses`, `rotation_stresses` |
| `test_solver.cpp` | `k_bc`, `f_bc` | `constrained_stiffness`, `constrained_loads` |
| `test_solver.cpp` | `p` | `applied_load` |
| `test_solver.cpp` | `delta` | `stretch` |
| `test_solver.cpp` | `th` | `rotation_angle` |
| `test_solver.cpp` | `m`, `n_dof` | `truss_model`, `num_dofs` |
| `test_vtk_writer.cpp` | `elems`, `u`, `stresses` (fixture) | `elements`, `displacements`, `axial_stresses` |
| `test_vtk_writer.cpp` | `path` | `output_path` |
| `test_vtk_writer.cpp` | `blocker` | `blocking_file` |
| `test_driver.cpp` | `log` | `log_stream` |
| `test_driver.cpp` | `rc` | `exit_code` |
| `test_driver.cpp` | `name` | `file_name` |
| `test_driver.cpp` | `v` | `component` |
| `test_driver.cpp` | `u_final`, `u_half`, `u_one` | `final_displacements`, `half_load_displacements`, `first_step_displacements` |
| `test_driver.cpp` | `i` | `index` |
| `test_driver.cpp` | `t` | `tolerance` |
