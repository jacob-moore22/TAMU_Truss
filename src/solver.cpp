#include "solver.h"

#include <cmath>
#include <stdexcept>

std::array<std::array<double, 4>, 4> k_local(const node &a, const node &b, double area, double e) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double l = std::sqrt(dx * dx + dy * dy);
    double c = dx / l;
    double s = dy / l;
    double k = e * area / l;

    std::array<std::array<double, 4>, 4> ke = {{
        {  k*c*c,  k*c*s, -k*c*c, -k*c*s },
        {  k*c*s,  k*s*s, -k*c*s, -k*s*s },
        { -k*c*c, -k*c*s,  k*c*c,  k*c*s },
        { -k*c*s, -k*s*s,  k*c*s,  k*s*s }  
    }};
    return ke;
}

matrix assemble(const std::vector<node> &nodes, const std::vector<elem> &elems) {
    int n_dof = 2 * (int)nodes.size();
    matrix k_global(n_dof, std::vector<double>(n_dof, 0.0));

    for (const auto &el : elems) {
        auto ke = k_local(nodes[el.n1 - 1], nodes[el.n2 - 1], el.a, el.e);
        int dofs[4] = { 2*(el.n1-1), 2*(el.n1-1)+1, 2*(el.n2-1), 2*(el.n2-1)+1 };
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                k_global[dofs[i]][dofs[j]] += ke[i][j];
            }
        }
    }
    return k_global;
}

std::vector<double> build_f(const std::vector<force> &forces, int n_dof) {
    std::vector<double> f_vec(n_dof, 0.0);
    for (const auto &fr : forces) {
        int dof = 2*(fr.node-1) + (fr.dof-1);
        f_vec[dof] += fr.val;
    }
    return f_vec;
}

void apply_bc(matrix &k_global, std::vector<double> &f_vec, const std::vector<bc> &bcs) {
    int n_dof = (int)f_vec.size();
    for (const auto &b : bcs) {
        int dof = 2*(b.node-1) + (b.dof-1);
        for (int i = 0; i < n_dof; ++i) {
            f_vec[i] -= k_global[i][dof] * b.val;
        }
        for (int i = 0; i < n_dof; ++i) {
            k_global[dof][i] = 0.0;
            k_global[i][dof] = 0.0;
        }
        k_global[dof][dof] = 1.0;
        f_vec[dof] = b.val;
    }
}

std::vector<double> gauss_solve(matrix k_global, std::vector<double> f_vec) {
    int n = (int)f_vec.size();

    for (int p = 0; p < n; ++p) {
        int max_row = p;
        double max_val = std::fabs(k_global[p][p]);
        for (int i = p + 1; i < n; ++i) {
            if (std::fabs(k_global[i][p]) > max_val) {
                max_val = std::fabs(k_global[i][p]);
                max_row = i;
            }
        }
        if (max_row != p) {
            std::swap(k_global[p], k_global[max_row]);
            std::swap(f_vec[p], f_vec[max_row]);
        }
        if (std::fabs(k_global[p][p]) < 1e-12) {
            throw std::runtime_error("singular stiffness matrix - check boundary conditions");
        }

        for (int i = p + 1; i < n; ++i) {
            double factor = k_global[i][p] / k_global[p][p];
            for (int j = p; j < n; ++j) {
                k_global[i][j] -= factor * k_global[p][j];
            }
            f_vec[i] -= factor * f_vec[p];
        }
    }

    std::vector<double> u_vec(n, 0.0);
    for (int i = n - 1; i >= 0; --i) {
        double sum = f_vec[i];
        for (int j = i + 1; j < n; ++j) {
            sum -= k_global[i][j] * u_vec[j];
        }
        u_vec[i] = sum / k_global[i][i];
    }
    return u_vec;
}

std::vector<double> reactions(const matrix &k_global, const std::vector<double> &u_vec, const std::vector<double> &f_vec) {
    int n = (int)u_vec.size();
    std::vector<double> r_vec(n, 0.0);
    for (int i = 0; i < n; ++i) {
        double sum = 0.0;
        for (int j = 0; j < n; ++j) {
            sum += k_global[i][j] * u_vec[j];
        }
        r_vec[i] = sum - f_vec[i];
    }
    return r_vec;
}

std::vector<double> elem_stress(const std::vector<node> &nodes, const std::vector<elem> &elems, const std::vector<double> &u_vec) {
    std::vector<double> stresses;
    stresses.reserve(elems.size());

    for (const auto &el : elems) {
        const node &a = nodes[el.n1 - 1];
        const node &b = nodes[el.n2 - 1];
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double l = std::sqrt(dx * dx + dy * dy);
        double c = dx / l;
        double s = dy / l;

        int dofs[4] = { 2*(el.n1-1), 2*(el.n1-1)+1, 2*(el.n2-1), 2*(el.n2-1)+1 };
        double ue[4] = { u_vec[dofs[0]], u_vec[dofs[1]], u_vec[dofs[2]], u_vec[dofs[3]] };

        double strain = (-c*ue[0] - s*ue[1] + c*ue[2] + s*ue[3]) / l;
        stresses.push_back(el.e * strain);
    }
    return stresses;
}
