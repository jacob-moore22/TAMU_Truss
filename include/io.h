/**
 * @file io.h
 * @brief Reading truss models from the plain-text input format.
 */
#pragma once

#include <string>

#include "types.h"

/**
 * @brief Parse a truss input file into a model.
 *
 * The file is split into sections introduced by the headers @c *NODES,
 * @c *ELEMENTS, @c *BOUNDARIES, @c *FORCES and @c *LOAD_STEPS (case
 * sensitive). Blank lines and lines starting with @c # are skipped. Node IDs
 * index directly into model::nodes; element IDs are read but ignored.
 *
 * @param input_path Path to the input file.
 * @return The parsed model. model::load_steps stays 1 if the file has no
 *         @c *LOAD_STEPS section.
 * @throws std::runtime_error If the file cannot be opened.
 */
model read_input(const std::string& input_path);
