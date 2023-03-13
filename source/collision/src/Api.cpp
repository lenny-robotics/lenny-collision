#include <lenny/collision/Api.h>
#include <lenny/collision/Solver.h>

namespace lenny::collision {

void Api::compute_T(Eigen::VectorXd& t, const PrimitiveInfo& pi_a, const PrimitiveInfo& pi_b, const bool& forFD) {
    Eigen::VectorXd states(pi_a.second.size() + pi_b.second.size());
    states << pi_a.second, pi_b.second;
    Solver solver(pi_a.first, pi_b.first, states);
    t = 0.5 * Eigen::VectorXd::Ones(solver.getSizeOfT());
    solver.compute_T(t, forFD);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> Api::computeClosestPoints(Eigen::VectorXd& t, const PrimitiveInfo& pi_a, const PrimitiveInfo& pi_b) {
    Eigen::VectorXd states(pi_a.second.size() + pi_b.second.size());
    states << pi_a.second, pi_b.second;
    Solver solver(pi_a.first, pi_b.first, states);
    return solver.computeClosestPoints(t);
}

double Api::compute_D(Eigen::VectorXd& t, const PrimitiveInfo& pi_a, const PrimitiveInfo& pi_b) {
    Eigen::VectorXd states(pi_a.second.size() + pi_b.second.size());
    states << pi_a.second, pi_b.second;
    Solver solver(pi_a.first, pi_b.first, states);
    return solver.compute_D(t);
}

void Api::compute_dDdS(Eigen::VectorXd& dDdS, Eigen::VectorXd& t, const PrimitiveInfo& pi_a, const PrimitiveInfo& pi_b) {
    Eigen::VectorXd states(pi_a.second.size() + pi_b.second.size());
    states << pi_a.second, pi_b.second;
    Solver solver(pi_a.first, pi_b.first, states);
    solver.compute_dDdS(dDdS, t);
}

void Api::compute_d2DdS2(Eigen::MatrixXd& d2DdS2, Eigen::VectorXd& t, const PrimitiveInfo& pi_a, const PrimitiveInfo& pi_b) {
    Eigen::VectorXd states(pi_a.second.size() + pi_b.second.size());
    states << pi_a.second, pi_b.second;
    Solver solver(pi_a.first, pi_b.first, states);
    solver.compute_d2DdS2(d2DdS2, t);
}

}  // namespace lenny::collision