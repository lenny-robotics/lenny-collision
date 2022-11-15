#pragma once

#include <lenny/tools/FiniteDifference.h>

#include <memory>

namespace lenny::collision {

class Parent {
public:
    //--- Pointers
    typedef std::shared_ptr<Parent> SPtr;
    typedef std::shared_ptr<const Parent> CSPtr;

    //--- Constructor
    Parent(const std::string& description) : description(description) {}
    virtual ~Parent() = default;

    //--- Compute point and its derivatives from primitive point
    virtual Eigen::Vector3d computePoint(const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const = 0;
    virtual void computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const = 0;
    virtual void computePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const = 0;

    void estimatePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const;
    void estimatePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const;

    void testPointJacobian(const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const;
    void testPointTensor(const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const;

    //--- Compute vector and its derivatives from primitive vector
    virtual Eigen::Vector3d computeVector(const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const = 0;
    virtual void computeVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const = 0;
    virtual void computeVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const = 0;

    void estimateVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const;
    void estimateVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const;

    void testVectorJacobian(const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const;
    void testVectorTensor(const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const;

    //--- Helpers
    virtual int getStateDimension() const = 0;

public:
    std::string description;  //Set by constructor
    bool useTensor = true;

protected:
    static tools::FiniteDifference fd;
};

}  // namespace lenny::collision
