#include <lenny/collision/RegularizerObjective.h>

namespace lenny::collision {

RegularizerObjective::RegularizerObjective() : optimization::Objective("RegularizerObjective") {}

double RegularizerObjective::computeValue(const Eigen::VectorXd& t) const {
    const Eigen::VectorXd diff = t - 0.5 * Eigen::VectorXd::Ones(t.size());
    return 0.5 * diff.dot(diff);
}

void RegularizerObjective::computeGradient(Eigen::VectorXd& pVpT, const Eigen::VectorXd& t) const {
    pVpT = t - 0.5 * Eigen::VectorXd::Ones(t.size());
}

void RegularizerObjective::computeHessian(Eigen::SparseMatrixD& p2VpT2, const Eigen::VectorXd& t) const {
    p2VpT2.resize(t.size(), t.size());
    p2VpT2.setIdentity();
}

}  // namespace lenny::collision