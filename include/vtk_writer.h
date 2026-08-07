#pragma once

#include <string>
#include <vector>
#include "types.h"

void write_vtk(
const std::string &path,
const std::vector<node> &nodes,
const std::vector<elem> &elems,
const std::vector<double> &u_vec,
const std::vector<double> &stresses);
