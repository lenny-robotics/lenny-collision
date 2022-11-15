#include <lenny/collision/Solver.h>
#include <lenny/optimization/Utils.h>
#include <lenny/tools/Logger.h>

namespace lenny::collision {

tools::FiniteDifference Solver::fd = tools::FiniteDifference("collision::Solver");

Solver::Solver(const Primitive::SPtr primitive_A, const Primitive::SPtr primitive_B, const Eigen::VectorXd& states)
    : objective(primitive_A, primitive_B, states) {
    optimizer.useSparseSolver = false;
    optimizer.printInfos = false;
}

void Solver::compute_T(Eigen::VectorXd& t, bool forFD) const {
    if (t.size() != objective.distanceCalculator.getTotalSizeOfT())
        LENNY_LOG_ERROR("Input t is incorrect!");
    if (t.size() == 0)
        return;

    const bool previousSetting = optimizer.printInfos;
    optimizer.printInfos = forFD ? false : previousSetting;
    optimizer.solverResidual = forFD ? 1e-12 : 1e-6;
    optimizer.optimize(t, objective, 100);
    optimizer.printInfos = previousSetting;
}

void Solver::compute_dTdS(Eigen::MatrixXd& dTdS, const Eigen::VectorXd& t) const {
    Eigen::MatrixXd p2VpT2(t.size(), t.size());
    objective.compute_p2VpT2(p2VpT2, t);

    Eigen::MatrixXd p2VpTpS(t.size(), objective.states.size());
    objective.compute_p2VpTpS(p2VpTpS, t);

    optimization::utils::solveLinearSystem(dTdS, p2VpT2, -p2VpTpS, "dTdS", true);
}

double Solver::compute_D(Eigen::VectorXd& t) const {
    ensureTandSareInSync(t);

    const double safetyMargins = objective.distanceCalculator.primitive_A->getSafetyMargin() + objective.distanceCalculator.primitive_B->getSafetyMargin();
    return objective.distanceCalculator.compute_D(objective.states, t) - safetyMargins * safetyMargins;
}

void Solver::compute_dDdS(Eigen::VectorXd& dDdS, Eigen::VectorXd& t) const {
    ensureTandSareInSync(t);

    Eigen::VectorXd pDpT(t.size());
    objective.distanceCalculator.compute_pDpT(pDpT, objective.states, t);

    Eigen::VectorXd pDpS(objective.states.size());
    objective.distanceCalculator.compute_pDpS(pDpS, objective.states, t);

    Eigen::MatrixXd dTdS(t.size(), objective.states.size());
    compute_dTdS(dTdS, t);

    dDdS = dTdS.transpose() * pDpT + pDpS;
}

/**
 * COMMENT: This is "only" the Gauss-Newton approximation of the Hessian, and not the "true Hessian"
 */
void Solver::compute_d2DdS2(Eigen::MatrixXd& d2DdS2, Eigen::VectorXd& t) const {
    ensureTandSareInSync(t);

    Eigen::MatrixXd p2DpT2(t.size(), t.size());
    objective.distanceCalculator.compute_p2DpT2(p2DpT2, objective.states, t);

    Eigen::MatrixXd p2DpS2(objective.states.size(), objective.states.size());
    objective.distanceCalculator.compute_p2DpS2(p2DpS2, objective.states, t);

    Eigen::MatrixXd p2DpTpS(t.size(), objective.states.size());
    objective.distanceCalculator.compute_p2DpTpS(p2DpTpS, objective.states, t);

    Eigen::MatrixXd dTdS(t.size(), objective.states.size());
    compute_dTdS(dTdS, t);

    d2DdS2 = (dTdS.transpose() * p2DpT2 + 2.0 * p2DpTpS.transpose()) * dTdS + p2DpS2;
}

void Solver::test_dTdS(const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::VectorXd& T, const Eigen::VectorXd& s) -> void {
        const Eigen::VectorXd states_tmp(objective.states);
        objective.states = s;
        compute_T(T, true);
        objective.states = states_tmp;
    };
    auto anal = [&](Eigen::MatrixXd& dTdS, const Eigen::VectorXd& s) -> void {
        const Eigen::VectorXd states_tmp(objective.states);
        objective.states = s;
        Eigen::VectorXd t_tmp(t);
        compute_T(t_tmp, true);
        compute_dTdS(dTdS, t_tmp);
        objective.states = states_tmp;
    };
    fd.testMatrix(eval, anal, objective.states, "dTdS", t.size(), true);
}

void Solver::test_dDdS(const Eigen::VectorXd& t) const {
    auto eval = [&](const Eigen::VectorXd& s) -> double {
        const Eigen::VectorXd states_tmp(objective.states);
        objective.states = s;
        Eigen::VectorXd t_tmp(t);
        compute_T(t_tmp, true);
        double D = compute_D(t_tmp);
        objective.states = states_tmp;
        return D;
    };
    auto anal = [&](Eigen::VectorXd& dDdS, const Eigen::VectorXd& s) -> void {
        const Eigen::VectorXd states_tmp(objective.states);
        objective.states = s;
        Eigen::VectorXd t_tmp(t);
        compute_T(t_tmp, true);
        compute_dDdS(dDdS, t_tmp);
        objective.states = states_tmp;
    };
    fd.testVector(eval, anal, objective.states, "dDdS");
}

void Solver::test_d2DdS2(const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::VectorXd& dDdS, const Eigen::VectorXd& s) -> void {
        const Eigen::VectorXd states_tmp(objective.states);
        objective.states = s;
        Eigen::VectorXd t_tmp(t);
        compute_T(t_tmp, true);
        compute_dDdS(dDdS, t_tmp);
        objective.states = states_tmp;
    };
    auto anal = [&](Eigen::MatrixXd& d2DdS2, const Eigen::VectorXd& s) -> void {
        const Eigen::VectorXd states_tmp(objective.states);
        objective.states = s;
        Eigen::VectorXd t_tmp(t);
        compute_T(t_tmp, true);
        compute_d2DdS2(d2DdS2, t_tmp);
        objective.states = states_tmp;
    };
    fd.testMatrix(eval, anal, objective.states, "d2DdS2", objective.states.size(), true);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> Solver::computeClosestPoints(Eigen::VectorXd& t) const {
    ensureTandSareInSync(t);
    return objective.distanceCalculator.compute_Ps(objective.states, t);
}

bool Solver::ensureTandSareInSync(Eigen::VectorXd& t) const {
    objective.distanceCalculator.checkInputs(objective.states, t);

    Eigen::VectorXd gradient(t.size());
    objective.computeGradient(gradient, t);
    if (gradient.norm() > 1e-5) {
        LENNY_LOG_WARNING("t and s should be in sync, but they are not... Making the corresponding computations again.")
        compute_T(t, false);
        return false;
    }
    return true;
}

int Solver::getSizeOFT() const {
    return objective.distanceCalculator.getTotalSizeOfT();
}

}  // namespace lenny::collision