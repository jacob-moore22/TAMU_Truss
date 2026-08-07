#include "vtk_writer.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>

void write_vtk(const std::string &path, const std::vector<node> &nodes, const std::vector<elem> &elems,
               const std::vector<double> &u_vec, const std::vector<double> &stresses) {
    std::filesystem::path p(path);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path());
    }

    std::ofstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("could not open output file: " + path);
    }

    int n_nodes = (int)nodes.size();
    int n_elems = (int)elems.size();

    f << "# vtk DataFile Version 3.0\n";
    f << "truss solver output\n";
    f << "ASCII\n";
    f << "DATASET UNSTRUCTURED_GRID\n";

    f << "POINTS " << n_nodes << " float\n";
    for (const auto &n : nodes) {
        f << n.x << " " << n.y << " 0.0\n";
    }

    f << "CELLS " << n_elems << " " << 3 * n_elems << "\n";
    for (const auto &el : elems) {
        f << "2 " << (el.n1 - 1) << " " << (el.n2 - 1) << "\n";
    }

    f << "CELL_TYPES " << n_elems << "\n";
    for (int i = 0; i < n_elems; ++i) {
        f << "3\n";
    }

    f << "POINT_DATA " << n_nodes << "\n";
    f << "VECTORS Displacement float\n";
    for (int i = 0; i < n_nodes; ++i) {
        f << u_vec[2 * i] << " " << u_vec[2 * i + 1] << " 0.0\n";
    }

    f << "CELL_DATA " << n_elems << "\n";
    f << "SCALARS Axial_Stress float 1\n";
    f << "LOOKUP_TABLE default\n";
    for (double s : stresses) {
        f << s << "\n";
    }
}
