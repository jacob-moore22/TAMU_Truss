#pragma once

#include <string>
#include <vector>

#include "types.h"

/**
 * @brief Write a legacy ASCII VTK unstructured grid file for one solution
 * state.
 *
 * Writes node geometry, element connectivity (as VTK line cells), nodal
 * displacements (as a point vector field), and per-element axial stress
 * (as a cell scalar field). Parent directories are created as needed.
 *
 * @param output_path Destination file path.
 * @param nodes Node positions.
 * @param elements Truss elements (connectivity).
 * @param displacements Nodal displacement vector, length 2*nodes.size().
 * @param stresses Per-element axial stress, in the same order as @p elements.
 * @throws std::runtime_error if the output file cannot be opened.
 */
void write_vtk(const std::string& output_path, const std::vector<node>& nodes,
               const std::vector<elem>& elements, const std::vector<double>& displacements,
               const std::vector<double>& stresses);
