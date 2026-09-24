/**
 * @file io.cpp
 * @brief Implementation of the input-file parser (see io.h).
 */
#include "io.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {

/// @brief Which block of the input file is currently being read.
///
/// Data lines are interpreted according to the most recent section
/// header seen.
enum class input_section {
    none,        ///< No section header seen yet
    nodes,       ///< After `*NODES`
    elements,    ///< After `*ELEMENTS`
    boundaries,  ///< After `*BOUNDARIES`
    forces,      ///< After `*FORCES`
    load_steps   ///< After `*LOAD_STEPS`
};

}  // namespace

model read_input(const std::string& path) {
    std::ifstream input_file(path);
    if (!input_file.is_open()) {
        throw std::runtime_error("could not open input file: " + path);
    }

    model truss;                                  // filled in below
    input_section section = input_section::none;  // no header seen yet
    std::string line;                             // one line of the file

    while (std::getline(input_file, line)) {
        // Peek at the first whitespace-separated token to decide whether the
        // line is blank, a comment, a section header, or data.
        std::istringstream first_token_stream(line);
        std::string first_token;
        first_token_stream >> first_token;

        if (first_token.empty() || first_token[0] == '#') continue;

        if (first_token == "*NODES") {
            section = input_section::nodes;
            continue;
        }
        if (first_token == "*ELEMENTS") {
            section = input_section::elements;
            continue;
        }
        if (first_token == "*BOUNDARIES") {
            section = input_section::boundaries;
            continue;
        }
        if (first_token == "*FORCES") {
            section = input_section::forces;
            continue;
        }
        if (first_token == "*LOAD_STEPS") {
            section = input_section::load_steps;
            continue;
        }

        // Data line: re-read the whole line as a stream of numbers.
        std::istringstream fields(line);
        if (section == input_section::nodes) {
            int node_id;  // 1-based node number
            double x, y;  // coordinates
            fields >> node_id >> x >> y;
            // Nodes may be listed in any order; grow the vector so that
            // node N always lands at index N-1.
            if (node_id > (int)truss.nodes.size()) truss.nodes.resize(node_id);
            truss.nodes[node_id - 1] = {x, y};
        } else if (section == input_section::elements) {
            int element_id;         // read but not stored; elements are kept
                                    // in file order
            int node_1, node_2;     // 1-based end nodes
            double area;            // cross-sectional area
            double youngs_modulus;  // material stiffness E
            fields >> element_id >> node_1 >> node_2 >> area >> youngs_modulus;
            truss.elements.push_back({node_1, node_2, area, youngs_modulus});
        } else if (section == input_section::boundaries) {
            int node_id;   // 1-based node with the support
            int dof;       // 1 = X, 2 = Y
            double value;  // prescribed displacement (0 = fixed)
            fields >> node_id >> dof >> value;
            truss.supports.push_back({node_id, dof, value});
        } else if (section == input_section::forces) {
            int node_id;   // 1-based node the force acts on
            int dof;       // 1 = X, 2 = Y
            double value;  // force magnitude, sign = direction
            fields >> node_id >> dof >> value;
            truss.loads.push_back({node_id, dof, value});
        } else if (section == input_section::load_steps) {
            truss.num_load_steps = std::stoi(first_token);
        }
        // Data before any section header is ignored.
    }

    return truss;
}
