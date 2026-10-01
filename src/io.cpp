/**
 * @file io.cpp
 * @brief Implementation of the truss input file parser.
 */
#include "io.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

model read_input(const std::string& input_path) {
    /// Input file being parsed.
    std::ifstream input_file(input_path);
    if (!input_file.is_open()) {
        throw std::runtime_error("could not open input file: " + input_path);
    }

    /// Model populated as sections are read.
    model truss_model;
    /// Current raw line of the input file.
    std::string line;
    /// Section being parsed: 0=none, 1=nodes, 2=elements, 3=boundaries,
    /// 4=forces, 5=load_steps.
    int current_section = 0;

    while (std::getline(input_file, line)) {
        /// Stream used to peek at the first token of the line.
        std::istringstream line_stream(line);
        /// First whitespace-delimited token (section header, comment or data).
        std::string first_token;
        line_stream >> first_token;

        if (first_token.empty() || first_token[0] == '#') continue;

        if (first_token == "*NODES") {
            current_section = 1;
            continue;
        }
        if (first_token == "*ELEMENTS") {
            current_section = 2;
            continue;
        }
        if (first_token == "*BOUNDARIES") {
            current_section = 3;
            continue;
        }
        if (first_token == "*FORCES") {
            current_section = 4;
            continue;
        }
        if (first_token == "*LOAD_STEPS") {
            current_section = 5;
            continue;
        }

        /// Stream reading all fields of a data line from the start.
        std::istringstream field_stream(line);
        if (current_section == 1) {
            int node_id;
            double x_coord, y_coord;
            field_stream >> node_id >> x_coord >> y_coord;
            if (node_id > (int)truss_model.nodes.size()) {
                truss_model.nodes.resize(node_id);
            }
            truss_model.nodes[node_id - 1] = {x_coord, y_coord};
        } else if (current_section == 2) {
            int element_id, start_node, end_node;
            double area, youngs_modulus;
            field_stream >> element_id >> start_node >> end_node >> area >>
                youngs_modulus;
            truss_model.elements.push_back(
                {start_node, end_node, area, youngs_modulus});
        } else if (current_section == 3) {
            int node_id, direction;
            double displacement;
            field_stream >> node_id >> direction >> displacement;
            truss_model.boundary_conditions.push_back(
                {node_id, direction, displacement});
        } else if (current_section == 4) {
            int node_id, direction;
            double magnitude;
            field_stream >> node_id >> direction >> magnitude;
            truss_model.forces.push_back({node_id, direction, magnitude});
        } else if (current_section == 5) {
            truss_model.load_steps = std::stoi(first_token);
        }
    }

    return truss_model;
}
