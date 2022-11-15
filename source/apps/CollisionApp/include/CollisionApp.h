#pragma once

#include <lenny/collision/Primitives.h>
#include <lenny/gui/Application.h>

#include <array>

#include "EulerAngleRigidBodyParent.h"

namespace lenny {

class CollisionApp : public gui::Application {
public:
    CollisionApp();
    ~CollisionApp() = default;

    //--- Drawing
    void drawScene() const override;
    void drawGui() override;

    //--- Interaction
    void mouseButtonCallback(double xPos, double yPos, int button, int action) override;

public:
    EulerAngleRigidBodyParent::SPtr parent = std::make_shared<EulerAngleRigidBodyParent>();

    enum RIGIDBODY { A, B };
    std::array<Eigen::VectorXd, 2> rbStates = {Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)};

    enum PRIMITIVES { SPHERE, CAPSULE, RECTANGLE, BOX };
    std::array<collision::Primitive::SPtr, 4> primitives = {
        std::make_shared<collision::Sphere>(parent, Eigen::Vector3d::Zero(), 0.5),
        std::make_shared<collision::Capsule>(parent, Eigen::Vector3d(0.0, -0.25, 0.0), Eigen::Vector3d(0.0, 0.25, 0.0), 0.25),
        std::make_shared<collision::Rectangle>(parent, Eigen::Vector3d::Zero(), Eigen::QuaternionD::Identity(), Eigen::Vector2d(0.5, 0.5), 0.01),
        std::make_shared<collision::Box>(parent, Eigen::Vector3d::Zero(), Eigen::QuaternionD::Identity(), Eigen::Vector3d(0.5, 0.5, 0.5), 0.01)};

    std::array<uint, 2> primitiveIndices = {BOX, BOX};

    Eigen::VectorXd* selectedRBState = nullptr;
};

}  // namespace lenny