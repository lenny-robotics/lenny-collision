#pragma once

#include <lenny/optimization/InequalityConstraint.h>

namespace lenny::collision {

class ParameterConstraint : public optimization::InequalityConstraint {
public:
    ParameterConstraint();
    ~ParameterConstraint() = default;

    uint getConstraintNumber() const override;
    void computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& t) const override;
    void computeJacobian(Eigen::SparseMatrixD& pCpT, const Eigen::VectorXd& t) const override;
    void computeTensor(Eigen::TensorD& p2CpT2, const Eigen::VectorXd& t) const override;

private:
    void setConstraintNumber(const Eigen::VectorXd& t) const;

private:
    mutable uint num_C = 0;
};

}  // namespace lenny::collision