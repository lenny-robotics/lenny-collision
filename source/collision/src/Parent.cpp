#include <lenny/collision/Parent.h>

namespace lenny::collision {

tools::FiniteDifference Parent::fd = tools::FiniteDifference("collision::Parent");

void Parent::estimatePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    auto eval = [&](Eigen::VectorXd& point, const Eigen::VectorXd& state) -> void { point = computePoint(state, p_primitive); };
    fd.estimateMatrix(jacobian, state, eval, 3, true);
}

void Parent::estimatePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    auto eval = [&](Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state) -> void { computePointJacobian(jacobian, state, p_primitive); };
    fd.estimateTensor(tensor, state, eval, 3, state.size());
}

bool Parent::testPointJacobian(const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    auto eval = [&](Eigen::VectorXd& point, const Eigen::VectorXd& state) -> void { point = computePoint(state, p_primitive); };
    auto anal = [&](Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state) -> void { computePointJacobian(jacobian, state, p_primitive); };
    return fd.testMatrix(eval, anal, state, "Point Jacobian", 3, true);
}

bool Parent::testPointTensor(const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    auto eval = [&](Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state) -> void { computePointJacobian(jacobian, state, p_primitive); };
    auto anal = [&](Eigen::TensorD& tensor, const Eigen::VectorXd& state) -> void { computePointTensor(tensor, state, p_primitive); };
    return fd.testTensor(eval, anal, state, "Point Tensor", 3, state.size());
}

void Parent::estimateVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    auto eval = [&](Eigen::VectorXd& vector, const Eigen::VectorXd& state) -> void { vector = computeVector(state, v_primitive); };
    fd.estimateMatrix(jacobian, state, eval, 3, true);
}

void Parent::estimateVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    auto eval = [&](Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state) -> void { computeVectorJacobian(jacobian, state, v_primitive); };
    fd.estimateTensor(tensor, state, eval, 3, state.size());
}

bool Parent::testVectorJacobian(const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    auto eval = [&](Eigen::VectorXd& vector, const Eigen::VectorXd& state) -> void { vector = computeVector(state, v_primitive); };
    auto anal = [&](Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state) -> void { computeVectorJacobian(jacobian, state, v_primitive); };
    return fd.testMatrix(eval, anal, state, "Vector Jacobian", 3, true);
}

bool Parent::testVectorTensor(const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    auto eval = [&](Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state) -> void { computeVectorJacobian(jacobian, state, v_primitive); };
    auto anal = [&](Eigen::TensorD& tensor, const Eigen::VectorXd& state) -> void { computeVectorTensor(tensor, state, v_primitive); };
    return fd.testTensor(eval, anal, state, "Vector Tensor", 3, state.size());
}

}  // namespace lenny::collision