#include <lenny/collision/DistanceCalculator.h>
#include <lenny/tools/Logger.h>

namespace lenny::collision {

tools::FiniteDifference DistanceCalculator::fd = tools::FiniteDifference("collision::DistanceCalculator");

DistanceCalculator::DistanceCalculator(const Primitive::SPtr primitive_A, const Primitive::SPtr primitive_B)
    : primitive_A(primitive_A), primitive_B(primitive_B) {}

double DistanceCalculator::compute_D(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);
    const auto [P_A, P_B] = compute_Ps(s, t);
    return (P_A - P_B).squaredNorm();
}

void DistanceCalculator::compute_pDpS(Eigen::VectorXd& pDpS, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);

    const auto [P_A, P_B] = compute_Ps(s, t);
    const auto [SIZE_S_A, SIZE_S_B] = getSizesOfS();
    const auto [s_A, s_B] = getSs(s);
    const auto [t_A, t_B] = getTs(t);

    Eigen::MatrixXd pPpS_A, pPpS_B;
    primitive_A->compute_pPpS(pPpS_A, s_A, t_A);
    primitive_B->compute_pPpS(pPpS_B, s_B, t_B);

    pDpS.resize(s.size());
    pDpS.setZero();
    pDpS.segment(0, SIZE_S_A) = 2.0 * pPpS_A.transpose() * (P_A - P_B);
    pDpS.segment(SIZE_S_A, SIZE_S_B) = -2.0 * pPpS_B.transpose() * (P_A - P_B);
}

void DistanceCalculator::compute_p2DpS2(Eigen::MatrixXd& p2DpS2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);

    const auto [P_A, P_B] = compute_Ps(s, t);
    const auto [SIZE_S_A, SIZE_S_B] = getSizesOfS();
    const auto [s_A, s_B] = getSs(s);
    const auto [t_A, t_B] = getTs(t);

    Eigen::MatrixXd pPpS_A, pPpS_B;
    primitive_A->compute_pPpS(pPpS_A, s_A, t_A);
    primitive_B->compute_pPpS(pPpS_B, s_B, t_B);

    Eigen::TensorD p2PpS2_A, p2PpS2_B;
    primitive_A->compute_p2PpS2(p2PpS2_A, s_A, t_A);
    primitive_B->compute_p2PpS2(p2PpS2_B, s_B, t_B);

    p2DpS2.resize(s.size(), s.size());
    p2DpS2.setZero();

    p2DpS2.block(0, 0, SIZE_S_A, SIZE_S_A) = 2.0 * pPpS_A.transpose() * pPpS_A;
    p2DpS2.block(SIZE_S_A, 0, SIZE_S_B, SIZE_S_A) = -2.0 * pPpS_B.transpose() * pPpS_A;
    p2DpS2.block(0, SIZE_S_A, SIZE_S_A, SIZE_S_B) = -2.0 * pPpS_A.transpose() * pPpS_B;
    p2DpS2.block(SIZE_S_A, SIZE_S_A, SIZE_S_B, SIZE_S_B) = 2.0 * pPpS_B.transpose() * pPpS_B;

    if (p2PpS2_A.getNumberOfEntries() > 0) {
        for (int i = 0; i < SIZE_S_A; i++) {
            Eigen::SparseMatrixD mat;
            p2PpS2_A.getMatrixForOuterIndex(mat, i);
            p2DpS2.block(0, i, SIZE_S_A, 1) += mat * 2.0 * (P_A - P_B);
        }
    }

    if (p2PpS2_B.getNumberOfEntries() > 0) {
        for (int i = 0; i < SIZE_S_B; i++) {
            Eigen::SparseMatrixD mat;
            p2PpS2_B.getMatrixForOuterIndex(mat, i);
            p2DpS2.block(SIZE_S_A, SIZE_S_A + i, SIZE_S_B, 1) -= mat * 2.0 * (P_A - P_B);
        }
    }
}

void DistanceCalculator::compute_pDpT(Eigen::VectorXd& pDpT, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);

    const auto [P_A, P_B] = compute_Ps(s, t);
    const auto [SIZE_T_A, SIZE_T_B] = getSizesOfT();
    const auto [s_A, s_B] = getSs(s);
    const auto [t_A, t_B] = getTs(t);

    Eigen::MatrixXd pPpT_A, pPpT_B;
    primitive_A->compute_pPpT(pPpT_A, s_A, t_A);
    primitive_B->compute_pPpT(pPpT_B, s_B, t_B);

    pDpT.resize(t.size());
    pDpT.setZero();
    pDpT.segment(0, SIZE_T_A) = 2.0 * pPpT_A.transpose() * (P_A - P_B);
    pDpT.segment(SIZE_T_A, SIZE_T_B) = -2.0 * pPpT_B.transpose() * (P_A - P_B);
}

void DistanceCalculator::compute_p2DpT2(Eigen::MatrixXd& p2DpT2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);

    const auto [P_A, P_B] = compute_Ps(s, t);
    const auto [SIZE_T_A, SIZE_T_B] = getSizesOfT();
    const auto [s_A, s_B] = getSs(s);
    const auto [t_A, t_B] = getTs(t);

    Eigen::MatrixXd pPpT_A, pPpT_B;
    primitive_A->compute_pPpT(pPpT_A, s_A, t_A);
    primitive_B->compute_pPpT(pPpT_B, s_B, t_B);

    Eigen::TensorD p2PpT2_A, p2PpT2_B;
    primitive_A->compute_p2PpT2(p2PpT2_A, s_A, t_A);
    primitive_B->compute_p2PpT2(p2PpT2_B, s_B, t_B);

    p2DpT2.resize(t.size(), t.size());
    p2DpT2.setZero();

    p2DpT2.block(0, 0, SIZE_T_A, SIZE_T_A) = 2.0 * pPpT_A.transpose() * pPpT_A;
    p2DpT2.block(SIZE_T_A, 0, SIZE_T_B, SIZE_T_A) = -2.0 * pPpT_B.transpose() * pPpT_A;
    p2DpT2.block(0, SIZE_T_A, SIZE_T_A, SIZE_T_B) = -2.0 * pPpT_A.transpose() * pPpT_B;
    p2DpT2.block(SIZE_T_A, SIZE_T_A, SIZE_T_B, SIZE_T_B) = 2.0 * pPpT_B.transpose() * pPpT_B;

    if (p2PpT2_A.getNumberOfEntries() > 0) {
        for (int i = 0; i < SIZE_T_A; i++) {
            Eigen::SparseMatrixD mat;
            p2PpT2_A.getMatrixForOuterIndex(mat, i);
            p2DpT2.block(0, i, SIZE_T_A, 1) += mat * 2.0 * (P_A - P_B);
        }
    }

    if (p2PpT2_B.getNumberOfEntries() > 0) {
        for (int i = 0; i < SIZE_T_B; i++) {
            Eigen::SparseMatrixD mat;
            p2PpT2_B.getMatrixForOuterIndex(mat, i);
            p2DpT2.block(SIZE_T_A, SIZE_T_A + i, SIZE_T_B, 1) -= mat * 2.0 * (P_A - P_B);
        }
    }
}

void DistanceCalculator::compute_p2DpTpS(Eigen::MatrixXd& p2DpTpS, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);

    const auto [P_A, P_B] = compute_Ps(s, t);
    const auto [SIZE_S_A, SIZE_S_B] = getSizesOfS();
    const auto [SIZE_T_A, SIZE_T_B] = getSizesOfT();
    const auto [s_A, s_B] = getSs(s);
    const auto [t_A, t_B] = getTs(t);

    Eigen::MatrixXd pPpT_A, pPpT_B;
    primitive_A->compute_pPpT(pPpT_A, s_A, t_A);
    primitive_B->compute_pPpT(pPpT_B, s_B, t_B);

    Eigen::MatrixXd pPpS_A, pPpS_B;
    primitive_A->compute_pPpS(pPpS_A, s_A, t_A);
    primitive_B->compute_pPpS(pPpS_B, s_B, t_B);

    Eigen::TensorD p2PpSpT_A, p2PpSpT_B;
    primitive_A->compute_p2PpSpT(p2PpSpT_A, s_A, t_A);
    primitive_B->compute_p2PpSpT(p2PpSpT_B, s_B, t_B);

    p2DpTpS.resize(t.size(), s.size());
    p2DpTpS.setZero();

    for (int a = 0; a < SIZE_T_A; a++) {
        const Eigen::Vector3d pPpT_A_part = pPpT_A.block(0, a, 3, 1);
        Eigen::SparseMatrixD mat;
        p2PpSpT_A.getMatrixForOuterIndex(mat, a);

        p2DpTpS.block(a, 0, 1, SIZE_S_A) = 2.0 * ((mat * (P_A - P_B)).transpose() + pPpT_A_part.transpose() * pPpS_A);
        p2DpTpS.block(a, SIZE_S_A, 1, SIZE_S_B) = -2.0 * pPpT_A_part.transpose() * pPpS_B;
    }

    for (int b = 0; b < SIZE_T_B; b++) {
        const Eigen::Vector3d pPpT_B_part = pPpT_B.block(0, b, 3, 1);
        Eigen::SparseMatrixD mat;
        p2PpSpT_B.getMatrixForOuterIndex(mat, b);

        p2DpTpS.block(SIZE_T_A + b, 0, 1, SIZE_S_A) = -2.0 * pPpT_B_part.transpose() * pPpS_A;
        p2DpTpS.block(SIZE_T_A + b, SIZE_S_A, 1, SIZE_S_B) = 2.0 * (-(mat * (P_A - P_B)).transpose() + pPpT_B_part.transpose() * pPpS_B);
    }
}

bool DistanceCalculator::test_pDpS(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](const Eigen::VectorXd& x) -> double { return compute_D(x, t); };
    auto anal = [&](Eigen::VectorXd& pDpS, const Eigen::VectorXd& x) -> void { compute_pDpS(pDpS, x, t); };
    return fd.testVector(eval, anal, s, "pDpS");
}

bool DistanceCalculator::test_p2DpS2(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::VectorXd& pDpS, const Eigen::VectorXd& x) -> void { compute_pDpS(pDpS, x, t); };
    auto anal = [&](Eigen::MatrixXd& p2DpS2, const Eigen::VectorXd& x) -> void { compute_p2DpS2(p2DpS2, x, t); };
    return fd.testMatrix(eval, anal, s, "p2DpS2", s.size(), true);
}

bool DistanceCalculator::test_pDpT(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](const Eigen::VectorXd& x) -> double { return compute_D(s, x); };
    auto anal = [&](Eigen::VectorXd& pDpT, const Eigen::VectorXd& x) -> void { compute_pDpT(pDpT, s, x); };
    return fd.testVector(eval, anal, t, "pDpT");
}

bool DistanceCalculator::test_p2DpT2(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::VectorXd& pDpT, const Eigen::VectorXd& x) -> void { compute_pDpT(pDpT, s, x); };
    auto anal = [&](Eigen::MatrixXd& p2DpT2, const Eigen::VectorXd& x) -> void { compute_p2DpT2(p2DpT2, s, x); };
    return fd.testMatrix(eval, anal, t, "p2DpT2", t.size(), true);
}

bool DistanceCalculator::test_p2DpTpS(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::VectorXd& pDpT, const Eigen::VectorXd& x) -> void { compute_pDpT(pDpT, x, t); };
    auto anal = [&](Eigen::MatrixXd& p2DpTpS, const Eigen::VectorXd& x) -> void { compute_p2DpTpS(p2DpTpS, x, t); };
    return fd.testMatrix(eval, anal, s, "p2DpTpS", t.size(), true);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> DistanceCalculator::compute_Ps(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    const auto [s_A, s_B] = getSs(s);
    const auto [t_A, t_B] = getTs(t);
    return {primitive_A->compute_P(s_A, t_A), primitive_B->compute_P(s_B, t_B)};
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> DistanceCalculator::getTs(const Eigen::VectorXd& t) const {
    const auto [SIZE_A, SIZE_B] = getSizesOfT();
    return {t.segment(0, SIZE_A), t.segment(SIZE_A, SIZE_B)};
}

std::pair<int, int> DistanceCalculator::getSizesOfT() const {
    return {primitive_A->getSizeOfT(), primitive_B->getSizeOfT()};
}

int DistanceCalculator::getTotalSizeOfT() const {
    return primitive_A->getSizeOfT() + primitive_B->getSizeOfT();
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> DistanceCalculator::getSs(const Eigen::VectorXd& s) const {
    const auto [SIZE_A, SIZE_B] = getSizesOfS();
    return {s.segment(0, SIZE_A), s.segment(SIZE_A, SIZE_B)};
}

std::pair<int, int> DistanceCalculator::getSizesOfS() const {
    return {primitive_A->getSizeOfS(), primitive_B->getSizeOfS()};
}

int DistanceCalculator::getTotalSizeOfS() const {
    return primitive_A->getSizeOfS() + primitive_B->getSizeOfS();
}

void DistanceCalculator::checkInputs(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    if (s.size() != getTotalSizeOfS() || t.size() != getTotalSizeOfT())
        LENNY_LOG_ERROR("Inputs are not correct!");
}

}  // namespace lenny::collision