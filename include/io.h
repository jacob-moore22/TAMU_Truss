/**
 * \file io.h
 * \brief Parser for the plain-text truss input file format.
 */
#pragma once

#include <string>
#include "types.h"

/**
 * \brief Parse a truss model from a plain-text input file.
 *
 * The file is a sequence of section headers followed by whitespace-separated
 * data rows: `*NODES` (`id x y`), `*ELEMENTS` (`id node1 node2 area
 * youngs_modulus`), `*BOUNDARIES` (`node dof value`), `*FORCES` (`node dof
 * value`), and `*LOAD_STEPS` (`n`). Blank lines and lines starting with `#`
 * are ignored. `*LOAD_STEPS` is optional; `model::load_steps` defaults to 1
 * when omitted. See the example files under `examples/` for complete examples.
 *
 * \param path Path to the input file.
 * \return The parsed model.
 * \throws std::runtime_error if the file cannot be opened.
 */
model read_input(const std::string &path);
