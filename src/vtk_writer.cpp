#include "vtk_writer.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>

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
               const std::vector<double>& stresses) {
    std::filesystem::path output_fs_path(output_path);
    if (output_fs_path.has_parent_path()) {
        std::filesystem::create_directories(output_fs_path.parent_path());
    }

    std::ofstream output_file(output_path);
    if (!output_file.is_open()) {
        throw std::runtime_error("could not open output file: " + output_path);
    }

    int node_count = (int)nodes.size();
    int element_count = (int)elements.size();

    output_file << "# vtk DataFile Version 3.0\n";
    output_file << "truss solver output\n";
    output_file << "ASCII\n";
    output_file << "DATASET UNSTRUCTURED_GRID\n";

    output_file << "POINTS " << node_count << " float\n";
    for (const auto& node_position : nodes) {
        output_file << node_position.x << " " << node_position.y << " 0.0\n";
    }

    output_file << "CELLS " << element_count << " " << 3 * element_count << "\n";
    for (const auto& element : elements) {
        output_file << "2 " << (element.node1_id - 1) << " " << (element.node2_id - 1) << "\n";
    }

    output_file << "CELL_TYPES " << element_count << "\n";
    for (int i = 0; i < element_count; ++i) {
        output_file << "3\n";
    }

    output_file << "POINT_DATA " << node_count << "\n";
    output_file << "VECTORS Displacement float\n";
    for (int node_index = 0; node_index < node_count; ++node_index) {
        output_file << displacements[2 * node_index] << " " << displacements[2 * node_index + 1]
                    << " 0.0\n";
    }

    output_file << "CELL_DATA " << element_count << "\n";
    output_file << "SCALARS Axial_Stress float 1\n";
    output_file << "LOOKUP_TABLE default\n";
    for (double stress_value : stresses) {
        output_file << stress_value << "\n";
    }
}
