#include "io.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

/**
 * @brief Parse a truss input file into an in-memory model.
 *
 * Reads a plain-text file organized into `*NODES`, `*ELEMENTS`, `*BOUNDARIES`,
 * `*FORCES`, and `*LOAD_STEPS` sections. Blank lines and lines beginning with
 * `#` are ignored. Node and element IDs in the file are 1-indexed.
 *
 * @param input_path Path to the truss input file.
 * @return The parsed model.
 * @throws std::runtime_error if the file cannot be opened.
 */
model read_input(const std::string& input_path) {
    std::ifstream input_file(input_path);
    if (!input_file.is_open()) {
        throw std::runtime_error("could not open input file: " + input_path);
    }

    model parsed_model;
    std::string line;
    int current_section = 0;  // 0=none, 1=nodes, 2=elements, 3=boundaries, 4=forces, 5=load_steps

    while (std::getline(input_file, line)) {
        std::istringstream keyword_stream(line);
        std::string first_token;
        keyword_stream >> first_token;

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

        std::istringstream data_stream(line);
        if (current_section == 1) {
            int node_id;
            double x, y;
            data_stream >> node_id >> x >> y;
            if (node_id > (int)parsed_model.nodes.size()) parsed_model.nodes.resize(node_id);
            parsed_model.nodes[node_id - 1] = {x, y};
        } else if (current_section == 2) {
            int element_id, node1_id, node2_id;
            double cross_section_area, youngs_modulus;
            data_stream >> element_id >> node1_id >> node2_id >> cross_section_area >> youngs_modulus;
            parsed_model.elements.push_back({node1_id, node2_id, cross_section_area, youngs_modulus});
        } else if (current_section == 3) {
            int node_id, dof_index;
            double value;
            data_stream >> node_id >> dof_index >> value;
            parsed_model.boundary_conditions.push_back({node_id, dof_index, value});
        } else if (current_section == 4) {
            int node_id, dof_index;
            double value;
            data_stream >> node_id >> dof_index >> value;
            parsed_model.forces.push_back({node_id, dof_index, value});
        } else if (current_section == 5) {
            parsed_model.load_steps = std::stoi(first_token);
        }
    }

    return parsed_model;
}
