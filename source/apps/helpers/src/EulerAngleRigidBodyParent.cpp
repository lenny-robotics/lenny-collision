#include "EulerAngleRigidBodyParent.h"

namespace lenny {

const tools::EulerAngleRigidBody EulerAngleRigidBodyParent::rigidBody;

EulerAngleRigidBodyParent::EulerAngleRigidBodyParent() : lenny::collision::Parent("EulerAngleRigidBodyParent") {}

Eigen::Vector3d EulerAngleRigidBodyParent::computePoint(const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    return rigidBody.computeGlobalPoint(state, p_primitive);
}

void EulerAngleRigidBodyParent::computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    Eigen::Matrix<double, 3, 6> jac;
    rigidBody.computePointJacobian(jac, state, p_primitive);
    jacobian = jac;
}

void EulerAngleRigidBodyParent::computePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    rigidBody.computeVectorTensor(tensor, state, p_primitive);
}

Eigen::Vector3d EulerAngleRigidBodyParent::computeVector(const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    return rigidBody.computeGlobalVector(state, v_primitive);
}

void EulerAngleRigidBodyParent::computeVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    Eigen::Matrix<double, 3, 6> jac;
    rigidBody.computeVectorJacobian(jac, state, v_primitive);
    jacobian = jac;
}

void EulerAngleRigidBodyParent::computeVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    rigidBody.computeVectorTensor(tensor, state, v_primitive);
}

int EulerAngleRigidBodyParent::getStateDimension() const {
    return rigidBody.STATE_SIZE;
}

}  // namespace lenny