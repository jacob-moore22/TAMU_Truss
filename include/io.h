#pragma once

#include <string>

#include "types.h"

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
model read_input(const std::string& input_path);
