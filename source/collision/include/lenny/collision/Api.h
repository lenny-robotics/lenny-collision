#pragma once

#include <lenny/collision/Primitive.h>

namespace lenny::collision {

class Api {
private:
    Api() = default;
    ~Api() = default;

public:
    typedef std::pair<const Primitive::SPtr, const Eigen::VectorXd> PrimitiveInfo;  //[primitive, parentState]

    static void compute_T(Eigen::VectorXd& t, const PrimitiveInfo& pi_a, const PrimitiveInfo& pi_b, const bool& forFD = false);
    static std::pair<Eigen::Vector3d, Eigen::Vector3d> computeClosestPoints(Eigen::VectorXd& t, const PrimitiveInfo& pi_a, const PrimitiveInfo& pi_b);
    static double compute_D(Eigen::VectorXd& t, const PrimitiveInfo& pi_a, const PrimitiveInfo& pi_b);
    static void compute_dDdS(Eigen::VectorXd& dDdS, Eigen::VectorXd& t, const PrimitiveInfo& pi_a, const PrimitiveInfo& pi_b);
    static void compute_d2DdS2(Eigen::MatrixXd& d2DdS2, Eigen::VectorXd& t, const PrimitiveInfo& pi_a, const PrimitiveInfo& pi_b);
};

}  // namespace lenny::collision