#pragma once

#include <lenny/collision/Parent.h>
#include <lenny/tools/EulerAngleRigidBody.h>

namespace lenny {

class EulerAngleRigidBodyParent : public collision::Parent {
public:
    //--- Pointers
    typedef std::unique_ptr<EulerAngleRigidBodyParent> UPtr;
    typedef std::shared_ptr<EulerAngleRigidBodyParent> SPtr;

    //--- Constructor
    EulerAngleRigidBodyParent();
    ~EulerAngleRigidBodyParent() = default;

    //--- Compute point and its derivatives from primitive point
    Eigen::Vector3d computePoint(const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const override;
    void computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const override;
    void computePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const override;

    //--- Compute vector and its derivatives from primitive vector
    Eigen::Vector3d computeVector(const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const override;
    void computeVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const override;
    void computeVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const override;

    //--- Helpers
    int getStateDimension() const override;

public:
    static const tools::EulerAngleRigidBody rigidBody;
};

}  // namespace lenny