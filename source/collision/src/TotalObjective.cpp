
#include <lenny/collision/TotalObjective.h>

namespace lenny::collision {

TotalObjective::TotalObjective(const Primitive::SPtr primitive_A, const Primitive::SPtr primitive_B, const Eigen::VectorXd& states)
    : optimization::Objective("TotalObjective"), distanceCalculator(primitive_A, primitive_B), states(states) {}

double TotalObjective::computeValue(const Eigen::VectorXd& t) const {
    return distanceCalculator.compute_D(states, t) + regularizerWeight * regularizerObjective.computeValue(t) +
           constraintWeight * parameterConstraint.computeValue(t);
}

void TotalObjective::computeGradient(Eigen::VectorXd& pVpT, const Eigen::VectorXd& t) const {
    Eigen::VectorXd pVpT_part(t.size());
    distanceCalculator.compute_pDpT(pVpT_part, states, t);
    pVpT = pVpT_part;

    regularizerObjective.computeGradient(pVpT_part, t);
    pVpT += regularizerWeight * pVpT_part;

    parameterConstraint.computeGradient(pVpT_part, t);
    pVpT += constraintWeight * pVpT_part;
}

void TotalObjective::computeHessian(Eigen::SparseMatrixD& p2VpT2, const Eigen::VectorXd& t) const {
    Eigen::MatrixXd p2DpT2;
    distanceCalculator.compute_p2DpT2(p2DpT2, states, t);
    p2VpT2 = Eigen::MatrixXd(p2DpT2.triangularView<Eigen::Lower>()).sparseView();

    Eigen::SparseMatrixD p2VpT2_part(t.size(), t.size());
    regularizerObjective.computeHessian(p2VpT2_part, t);
    p2VpT2 += regularizerWeight * p2VpT2_part;

    parameterConstraint.computeHessian(p2VpT2_part, t);
    p2VpT2 += constraintWeight * p2VpT2_part;
}

void TotalObjective::compute_p2VpT2(Eigen::MatrixXd& p2VpT2, const Eigen::VectorXd& t) const {
    Eigen::SparseMatrixD p2VpT2_sparse;
    computeHessian(p2VpT2_sparse, t);
    p2VpT2 = p2VpT2_sparse.toDense();
}

void TotalObjective::compute_p2VpTpS(Eigen::MatrixXd& p2VpTpS, const Eigen::VectorXd& t) const {
    distanceCalculator.compute_p2DpTpS(p2VpTpS, states, t);
}

}  // namespace lenny::collision