/**
 * @file vtk_writer.h
 * @brief Writing results as legacy VTK files for ParaView.
 */
#pragma once

#include <string>
#include <vector>

#include "types.h"

/**
 * @brief Writes the deformed truss to a legacy ASCII VTK file.
 *
 * Nodes become points, elements become line cells, displacements are stored
 * as a point vector field `Displacement` and axial stresses as a cell scalar
 * field `Axial_Stress`. Parent directories of @p path are created if missing.
 *
 * @param path          Output file path.
 * @param nodes         All nodes of the truss.
 * @param elements      All elements of the truss.
 * @param displacements Two entries per node:
 *                      @f$[u_{1x}, u_{1y}, u_{2x}, u_{2y}, \ldots]@f$.
 * @param stresses      One entry per element, in element order.
 * @throws std::runtime_error if the file cannot be opened for writing.
 */
void write_vtk(const std::string& path, const std::vector<node>& nodes,
               const std::vector<element>& elements,
               const std::vector<double>& displacements,
               const std::vector<double>& stresses);
