/**
 * @file vtk_writer.h
 * @brief Writing truss results as legacy ASCII VTK for ParaView.
 */
#pragma once

#include <string>
#include <vector>

#include "types.h"

/**
 * @brief Write one result state to a legacy ASCII VTK unstructured grid.
 *
 * Nodes become points (z = 0), elements become VTK_LINE cells with 0-based
 * connectivity. Displacements are written as the point vector
 * @c Displacement and stresses as the cell scalar @c Axial_Stress.
 * Missing parent directories are created.
 *
 * @param output_path    Destination file path.
 * @param nodes          All nodes of the model.
 * @param elements       All elements of the model.
 * @param displacements  Global displacement vector (2 entries per node).
 * @param axial_stresses One stress per element.
 * @throws std::runtime_error If the file cannot be opened for writing.
 * @throws std::filesystem::filesystem_error If a parent directory cannot be
 *         created.
 */
void write_vtk(const std::string& output_path, const std::vector<node>& nodes,
               const std::vector<elem>& elements,
               const std::vector<double>& displacements,
               const std::vector<double>& axial_stresses);
