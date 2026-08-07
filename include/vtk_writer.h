/**
 * \file vtk_writer.h
 * \brief Writer for legacy VTK ASCII output, loadable directly in ParaView.
 */
#pragma once

#include "types.h"
#include <string>
#include <vector>

/**
 * \brief Write nodal displacements and per-element axial stress to a legacy
 *        VTK ASCII (`UNSTRUCTURED_GRID`) file.
 *
 * Emits nodal `Displacement` vectors (as `POINT_DATA`) and per-element
 * `Axial_Stress` scalars (as `CELL_DATA`). Any missing parent directories in
 * \p path are created automatically. In ParaView, use `WarpByVector` on
 * `Displacement` and color by `Axial_Stress`.
 *
 * \param path Output file path.
 * \param nodes All nodes in the model.
 * \param elems All elements in the model.
 * \param u_vec Nodal displacement vector (length `2*nodes.size()`).
 * \param stresses Per-element axial stress (same order as \p elems).
 * \throws std::runtime_error if the output file cannot be opened.
 */
void write_vtk(const std::string &path, const std::vector<node> &nodes, const std::vector<elem> &elems,
               const std::vector<double> &u_vec, const std::vector<double> &stresses);
