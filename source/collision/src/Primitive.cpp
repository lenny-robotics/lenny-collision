#include <lenny/collision/Primitive.h>
#include <lenny/tools/Logger.h>

#include <iostream>

namespace lenny::collision {

tools::FiniteDifference Primitive::fd = tools::FiniteDifference("collision::Primitive");

Primitive::Primitive(const std::string& description, const Parent::SPtr parent) : description(description), parent(parent) {}

Eigen::Vector3d Primitive::compute_P(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);
    Eigen::Vector3d P = parent->computePoint(s, point);
    for (int i = 0; i < t.size(); i++)
        P += t[i] * parent->computeVector(s, vectors[i]);
    return P;
}

void Primitive::compute_pPpS(Eigen::MatrixXd& pPpS, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);
    pPpS.resize(3, s.size());
    parent->computePointJacobian(pPpS, s, point);
    Eigen::MatrixXd jac_v(3, s.size());
    for (int i = 0; i < t.size(); i++) {
        parent->computeVectorJacobian(jac_v, s, vectors[i]);
        pPpS += t[i] * jac_v;
    }
}

void Primitive::compute_p2PpS2(Eigen::TensorD& p2PpS2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);
    p2PpS2.resize(Eigen::Vector3i(3, s.size(), s.size()));
    parent->computePointTensor(p2PpS2, s, point);
    Eigen::TensorD ten_v(Eigen::Vector3i(3, s.size(), s.size()));
    std::vector<std::pair<Eigen::Vector3i, double>> ten_v_entries;
    for (int i = 0; i < t.size(); i++) {
        parent->computeVectorTensor(ten_v, s, vectors[i]);
        ten_v.getEntryList(ten_v_entries);
        for (const auto& [index, value] : ten_v_entries)
            p2PpS2.addEntry(index, t[i] * value);
    }
}

void Primitive::compute_pPpT(Eigen::MatrixXd& pPpT, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);
    pPpT.resize(3, t.size());
    for (int i = 0; i < t.size(); i++)
        pPpT.col(i) = parent->computeVector(s, vectors[i]);
}

void Primitive::compute_p2PpT2(Eigen::TensorD& p2PpT2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);
    p2PpT2.resize(Eigen::Vector3i(3, t.size(), t.size()));
    p2PpT2.setZero();
}

void Primitive::compute_p2PpSpT(Eigen::TensorD& p2PpSpT, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    checkInputs(s, t);
    p2PpSpT.resize(Eigen::Vector3i(3, s.size(), t.size()));
    Eigen::MatrixXd jac_v(3, s.size());
    for (int i = 0; i < t.size(); i++) {
        parent->computeVectorJacobian(jac_v, s, vectors[i]);
        for (int k = 0; k < jac_v.outerSize(); ++k)
            for (Eigen::MatrixXd::InnerIterator it(jac_v, k); it; ++it)
                p2PpSpT.addEntry(Eigen::Vector3i(it.row(), it.col(), i), it.value());
    }
}

void Primitive::estimate_pPpS(Eigen::MatrixXd& pPpS, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::VectorXd& P, const Eigen::VectorXd& x) -> void { P = compute_P(x, t); };
    fd.estimateMatrix(pPpS, s, eval, 3, true);
}

void Primitive::estimate_p2PpS2(Eigen::TensorD& p2PpS2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::MatrixXd& pPpS, const Eigen::VectorXd& x) -> void { compute_pPpS(pPpS, x, t); };
    fd.estimateTensor(p2PpS2, s, eval, 3, s.size());
}

void Primitive::estimate_pPpT(Eigen::MatrixXd& pPpT, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::VectorXd& P, const Eigen::VectorXd& x) -> void { P = compute_P(s, x); };
    fd.estimateMatrix(pPpT, t, eval, 3, true);
}

void Primitive::estimate_p2PpT2(Eigen::TensorD& p2PpT2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::MatrixXd& pPpT, const Eigen::VectorXd& x) -> void { compute_pPpT(pPpT, s, x); };
    fd.estimateTensor(p2PpT2, t, eval, 3, t.size());
}

void Primitive::estimate_p2PpSpT(Eigen::TensorD& p2PpSpT, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::MatrixXd& pPpS, const Eigen::VectorXd& x) -> void { compute_pPpS(pPpS, s, x); };
    fd.estimateTensor(p2PpSpT, t, eval, 3, s.size());
}

bool Primitive::test_pPpS(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::VectorXd& P, const Eigen::VectorXd& x) -> void { P = compute_P(x, t); };
    auto anal = [&](Eigen::MatrixXd& pPpS, const Eigen::VectorXd& x) -> void { compute_pPpS(pPpS, x, t); };
    return fd.testMatrix(eval, anal, s, "pPpS", 3, true);
}

bool Primitive::test_p2PpS2(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::MatrixXd& pPpS, const Eigen::VectorXd& x) -> void { compute_pPpS(pPpS, x, t); };
    auto anal = [&](Eigen::TensorD& p2PpS2, const Eigen::VectorXd& x) -> void { compute_p2PpS2(p2PpS2, x, t); };
    return fd.testTensor(eval, anal, s, "p2PpS2", 3, s.size());
}

bool Primitive::test_pPpT(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::VectorXd& P, const Eigen::VectorXd& x) -> void { P = compute_P(s, x); };
    auto anal = [&](Eigen::MatrixXd& pPpT, const Eigen::VectorXd& x) -> void { compute_pPpT(pPpT, s, x); };
    return fd.testMatrix(eval, anal, t, "pPpT", 3, true);
}

bool Primitive::test_p2PpT2(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::MatrixXd& pPpT, const Eigen::VectorXd& x) -> void { compute_pPpT(pPpT, s, x); };
    auto anal = [&](Eigen::TensorD& p2PpT2, const Eigen::VectorXd& x) -> void { compute_p2PpT2(p2PpT2, s, x); };
    return fd.testTensor(eval, anal, t, "p2PpT2", 3, t.size());
}

bool Primitive::test_p2PpSpT(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    auto eval = [&](Eigen::MatrixXd& pPpS, const Eigen::VectorXd& x) -> void { compute_pPpS(pPpS, s, x); };
    auto anal = [&](Eigen::TensorD& p2PpSpT, const Eigen::VectorXd& x) -> void { compute_p2PpSpT(p2PpSpT, s, x); };
    return fd.testTensor(eval, anal, t, "p2PpSpT", 3, s.size());
}

int Primitive::getSizeOfS() const {
    return parent->getStateDimension();
}

int Primitive::getSizeOfT() const {
    return vectors.size();
}

double Primitive::getSafetyMargin(const double& D) const {
    return safetyMargin;
}

void Primitive::checkInputs(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const {
    if (s.size() != getSizeOfS() || t.size() != getSizeOfT())
        LENNY_LOG_ERROR("Invalid inputs-> Size s: %d VS %d / Size t: %d VS %d", s.size(), getSizeOfS(), t.size(), getSizeOfT());
}

void Primitive::printInfos() const {
    json js;
    to_json(js);
    std::cout << std::setw(2) << js << std::endl;
}

void Primitive::to_json(json& j) const {
    j["primitive-description"] = description;
    j["parent-description"] = parent->description;
}

void Primitive::from_json(const json& j) {
    description = j.value("primitive-description", std::string());
    parent->description = j.value("parent-description", std::string());
}

}  // namespace lenny::collision