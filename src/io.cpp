#include "io.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

model read_input(const std::string &path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("could not open input file: " + path);
    }

    model m;
    std::string line;
    int section = 0; // 0=none, 1=nodes, 2=elements, 3=boundaries, 4=forces, 5=load_steps

while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string first;
    ss >> first;

    if (first.empty() || first[0] == '#') continue;

    if (first == "*NODES") { section = 1; continue; }
    if (first == "*ELEMENTS") { section = 2; continue; }
    if (first == "*BOUNDARIES") { section = 3; continue; }
    if (first == "*FORCES") { section = 4; continue; }
    if (first == "*LOAD_STEPS") { section = 5; continue; }

    std::istringstream data(line);
    if (section == 1) {
        int id; double x, y;
        data >> id >> x >> y;
        if (id > (int)m.nodes.size()) m.nodes.resize(id);
        m.nodes[id - 1] = {x, y};
    } else if (section == 2) {
        int id, n1, n2; double a, e;
        data >> id >> n1 >> n2 >> a >> e;
        m.elems.push_back({n1, n2, a, e});
    } else if (section == 3) {
        int nd, dof; double val;
        data >> nd >> dof >> val;
        m.bcs.push_back({nd, dof, val});
    } else if (section == 4) {
        int nd, dof; double val;
        data >> nd >> dof >> val;
        m.forces.push_back({nd, dof, val});
    } else if (section == 5) {
        m.load_steps = std::stoi(first);
    }
    }

    return m;
}
