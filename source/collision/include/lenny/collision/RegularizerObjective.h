#include <lenny/optimization/Objective.h>

namespace lenny::collision {

class RegularizerObjective : public optimization::Objective {
public:
    RegularizerObjective();
    ~RegularizerObjective() = default;

    double computeValue(const Eigen::VectorXd& t) const override;
    void computeGradient(Eigen::VectorXd& pVpT, const Eigen::VectorXd& t) const override;
    void computeHessian(Eigen::SparseMatrixD& p2VpT2, const Eigen::VectorXd& t) const override;
};

}  // namespace lenny::collision
