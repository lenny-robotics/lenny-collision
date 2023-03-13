#pragma once

#include <lenny/collision/TotalObjective.h>
#include <lenny/optimization/NewtonOptimizer.h>

namespace lenny::collision {

class Solver {
public:
    //--- Constructor
    Solver(const Primitive::SPtr primitive_A, const Primitive::SPtr primitive_B, const Eigen::VectorXd& states);
    ~Solver() = default;

    //--- Compute t and derivative
    void compute_T(Eigen::VectorXd& t, bool forFD = false) const;
    void compute_dTdS(Eigen::MatrixXd& dTdS, const Eigen::VectorXd& t) const;

    //--- Compute D and derivatives
    double compute_D(const Eigen::VectorXd& t) const;
    void compute_dDdS(Eigen::VectorXd& dDdS, const Eigen::VectorXd& t) const;
    void compute_d2DdS2(Eigen::MatrixXd& d2DdS2, const Eigen::VectorXd& t) const;

    //--- Tests
    bool test_dTdS(const Eigen::VectorXd& t) const;
    bool test_dDdS(const Eigen::VectorXd& t) const;
    bool test_d2DdS2(const Eigen::VectorXd& t) const;

    //--- Helpers
    std::pair<Eigen::Vector3d, Eigen::Vector3d> computeClosestPoints(const Eigen::VectorXd& t) const;
    bool checkIfTandSAreInSync(const Eigen::VectorXd& t) const;
    int getSizeOfT() const;

public:
    //--- Members
    collision::TotalObjective objective;
    mutable optimization::NewtonOptimizer optimizer = optimization::NewtonOptimizer("CollisionSolver");
    inline static bool applyAdditionalProjectionStep = true; //If true, an additional projection step for box primitives is applied

private:
    static tools::FiniteDifference fd;
};

}  // namespace lenny::collision