#pragma once

#include <lenny/collision/DistanceCalculator.h>
#include <lenny/collision/ParameterConstraint.h>
#include <lenny/collision/RegularizerObjective.h>

namespace lenny::collision {

class TotalObjective : public optimization::Objective {
public:
    TotalObjective(const Primitive::SPtr primitive_A, const Primitive::SPtr primitive_B, const Eigen::VectorXd& states);
    ~TotalObjective() = default;

    double computeValue(const Eigen::VectorXd& t) const override;
    void computeGradient(Eigen::VectorXd& pVpT, const Eigen::VectorXd& t) const override;
    void computeHessian(Eigen::SparseMatrixD& p2VpT2, const Eigen::VectorXd& t) const override;

    void compute_p2VpT2(Eigen::MatrixXd& p2VpT2, const Eigen::VectorXd& t) const;
    void compute_p2VpTpS(Eigen::MatrixXd& p2VpTpS, const Eigen::VectorXd& t) const;

public:
    DistanceCalculator distanceCalculator;
    mutable Eigen::VectorXd states;

    RegularizerObjective regularizerObjective;
    double regularizerWeight = 0.01;

    ParameterConstraint parameterConstraint;
    double constraintWeight = 10.0;
};

}  // namespace lenny::collision