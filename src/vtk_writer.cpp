/**
 * @file vtk_writer.cpp
 * @brief Implementation of the legacy ASCII VTK writer.
 */
#include "vtk_writer.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>

void write_vtk(const std::string& output_path, const std::vector<node>& nodes,
               const std::vector<elem>& elements,
               const std::vector<double>& displacements,
               const std::vector<double>& axial_stresses) {
    /// Output path as a filesystem path, used to create parent directories.
    std::filesystem::path file_path(output_path);
    if (file_path.has_parent_path()) {
        std::filesystem::create_directories(file_path.parent_path());
    }

    /// Output VTK file stream.
    std::ofstream vtk_file(output_path);
    if (!vtk_file.is_open()) {
        throw std::runtime_error("could not open output file: " + output_path);
    }

    /// Number of points written.
    int num_nodes = (int)nodes.size();
    /// Number of line cells written.
    int num_elements = (int)elements.size();

    vtk_file << "# vtk DataFile Version 3.0\n";
    vtk_file << "truss solver output\n";
    vtk_file << "ASCII\n";
    vtk_file << "DATASET UNSTRUCTURED_GRID\n";

    vtk_file << "POINTS " << num_nodes << " float\n";
    for (const auto& point : nodes) {
        vtk_file << point.x << " " << point.y << " 0.0\n";
    }

    vtk_file << "CELLS " << num_elements << " " << 3 * num_elements << "\n";
    for (const auto& element : elements) {
        vtk_file << "2 " << (element.start_node - 1) << " "
                 << (element.end_node - 1) << "\n";
    }

    vtk_file << "CELL_TYPES " << num_elements << "\n";
    for (int cell_index = 0; cell_index < num_elements; ++cell_index) {
        vtk_file << "3\n";
    }

    vtk_file << "POINT_DATA " << num_nodes << "\n";
    vtk_file << "VECTORS Displacement float\n";
    for (int node_index = 0; node_index < num_nodes; ++node_index) {
        vtk_file << displacements[2 * node_index] << " "
                 << displacements[2 * node_index + 1] << " 0.0\n";
    }

    vtk_file << "CELL_DATA " << num_elements << "\n";
    vtk_file << "SCALARS Axial_Stress float 1\n";
    vtk_file << "LOOKUP_TABLE default\n";
    for (double stress : axial_stresses) {
        vtk_file << stress << "\n";
    }
}
