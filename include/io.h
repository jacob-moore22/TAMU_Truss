/**
 * @file io.h
 * @brief Reading truss input files.
 */
#pragma once

#include <string>

#include "types.h"

/**
 * @brief Reads a truss description from a plain-text file.
 *
 * The file is divided into sections by header lines. Blank lines and lines
 * starting with `#` are ignored. Sections may appear in any order:
 *
 * | Header        | Columns on each data line                          |
 * |---------------|----------------------------------------------------|
 * | `*NODES`      | id  x  y                                           |
 * | `*ELEMENTS`   | id  node_1  node_2  area  youngs_modulus           |
 * | `*BOUNDARIES` | node_id  dof (1 = X, 2 = Y)  prescribed_displacement |
 * | `*FORCES`     | node_id  dof (1 = X, 2 = Y)  force_value           |
 * | `*LOAD_STEPS` | number_of_load_steps                               |
 *
 * @param path Path to the input file.
 * @return The parsed truss model.
 * @throws std::runtime_error if the file cannot be opened.
 */
model read_input(const std::string& path);
