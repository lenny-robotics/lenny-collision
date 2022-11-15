#include <lenny/collision/ParameterConstraint.h>
#include <lenny/tools/Logger.h>

namespace lenny::collision {

ParameterConstraint::ParameterConstraint() : optimization::InequalityConstraint("ParameterConstraint") {}

uint ParameterConstraint::getConstraintNumber() const {
    return this->num_C;
}

void ParameterConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& t) const {
    this->num_C = 2 * t.size();
    C.resize(this->num_C);
    int iter = 0;
    for (int i = 0; i < t.size(); i++) {
        C[iter++] = -t[i] - 0.0;
        C[iter++] = t[i] - 1.0;
    }
}

void ParameterConstraint::computeJacobian(Eigen::SparseMatrixD& pCpT, const Eigen::VectorXd& t) const {
    this->num_C = 2 * t.size();
    pCpT.resize(this->num_C, t.size());
    pCpT.setZero();
    int iter = 0;
    for (int i = 0; i < t.size(); i++) {
        pCpT.coeffRef(iter++, i) = -1.0;
        pCpT.coeffRef(iter++, i) = 1.0;
    }
}

void ParameterConstraint::computeTensor(Eigen::TensorD& p2CpT2, const Eigen::VectorXd& t) const {
    this->num_C = 2 * t.size();
    p2CpT2.resize(Eigen::Vector3i(getConstraintNumber(), t.size(), t.size()));
}

}  // namespace lenny::collision