/**
 * @file vtk_writer.cpp
 * @brief Implementation of the legacy VTK writer (see vtk_writer.h).
 */
#include "vtk_writer.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>

void write_vtk(const std::string& path, const std::vector<node>& nodes,
               const std::vector<element>& elements,
               const std::vector<double>& displacements,
               const std::vector<double>& stresses) {
    // Make sure the folder we are writing into exists.
    std::filesystem::path output_path(path);
    if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
    }

    std::ofstream vtk_file(path);
    if (!vtk_file.is_open()) {
        throw std::runtime_error("could not open output file: " + path);
    }

    int num_nodes = (int)nodes.size();
    int num_elements = (int)elements.size();

    // ---- Header (legacy VTK ASCII format, version 3.0) ----
    vtk_file << "# vtk DataFile Version 3.0\n";
    vtk_file << "truss solver output\n";
    vtk_file << "ASCII\n";
    vtk_file << "DATASET UNSTRUCTURED_GRID\n";

    // ---- Geometry: one 3D point per node (z = 0 for a 2D truss) ----
    vtk_file << "POINTS " << num_nodes << " float\n";
    for (const auto& current_node : nodes) {
        vtk_file << current_node.x << " " << current_node.y << " 0.0\n";
    }

    // ---- Connectivity: one line cell per element ----
    // Each cell line is "2 i j": the count (2 points) followed by the
    // 0-based indices of the two end nodes. The second number on the CELLS
    // header is the total count of integers that follow (3 per element).
    vtk_file << "CELLS " << num_elements << " " << 3 * num_elements << "\n";
    for (const auto& el : elements) {
        vtk_file << "2 " << (el.node_1 - 1) << " " << (el.node_2 - 1) << "\n";
    }

    // VTK cell type 3 = VTK_LINE.
    vtk_file << "CELL_TYPES " << num_elements << "\n";
    for (int i = 0; i < num_elements; ++i) {
        vtk_file << "3\n";
    }

    // ---- Results attached to nodes: displacement vector (z = 0) ----
    vtk_file << "POINT_DATA " << num_nodes << "\n";
    vtk_file << "VECTORS Displacement float\n";
    for (int i = 0; i < num_nodes; ++i) {
        vtk_file << displacements[2 * i] << " " << displacements[2 * i + 1]
                 << " 0.0\n";
    }

    // ---- Results attached to elements: axial stress scalar ----
    vtk_file << "CELL_DATA " << num_elements << "\n";
    vtk_file << "SCALARS Axial_Stress float 1\n";
    vtk_file << "LOOKUP_TABLE default\n";
    for (double stress : stresses) {
        vtk_file << stress << "\n";
    }
}
