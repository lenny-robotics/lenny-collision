#include "CollisionApp.h"

#include <lenny/collision/Solver.h>
#include <lenny/gui/Guizmo.h>
#include <lenny/gui/ImGui.h>
#include <lenny/gui/Model.h>
#include <lenny/gui/Renderer.h>

namespace lenny {

CollisionApp::CollisionApp() : gui::Application("CollisionApp") {
    //Setup scene
    const auto [width, height] = getCurrentWindowSize();
    scenes.emplace_back(std::make_shared<gui::Scene>("Scene-1", width, height));
    scenes.back()->f_drawScene = [&]() -> void { drawScene(); };
    scenes.back()->f_mouseButtonCallback = [&](double xPos, double yPos, Ray ray, int button, int action) -> void {
        mouseButtonCallback(xPos, yPos, ray, button, action);
    };
    scenes.back()->showOrigin = false;
    scenes.back()->showGround = false;

    //Initialize state
    rbStates[B][1] = 1.0;
}

void CollisionApp::drawScene() const {
    //Create collision solver
    Eigen::VectorXd states(rbStates[A].size() + rbStates[B].size());
    states << rbStates[A], rbStates[B];
    collision::Solver solver(primitives[primitiveIndices[A]], primitives[primitiveIndices[B]], states);

    //Compute t
    Eigen::VectorXd t = 0.5 * Eigen::VectorXd::Ones(solver.getSizeOFT());
    solver.compute_T(t);

    //Compute shortest distance
    const double distance = solver.compute_D(t);
    LENNY_LOG_PRINT(tools::Logger::DEFAULT, "Shortest Distance (squared): %lf\n", distance);

    //Draw closest points and connection line
    const auto [P_A, P_B] = solver.computeClosestPoints(t);
    gui::Renderer::I->drawCylinder(P_A, P_B, 0.025, Eigen::Vector4d(0.0, 0.75, 0.0, 0.75));
    gui::Renderer::I->drawSphere(P_A, 0.05, Eigen::Vector4d(0.0, 0.0, 0.75, 0.75));
    gui::Renderer::I->drawSphere(P_B, 0.05, Eigen::Vector4d(0.0, 0.0, 0.75, 0.75));

    //Draw primitives
    primitives[primitiveIndices[A]]->drawScene(rbStates[A], Eigen::Vector4d(0.75, 0.0, 0.0, 0.5));
    primitives[primitiveIndices[B]]->drawScene(rbStates[B], Eigen::Vector4d(0.75, 0.0, 0.0, 0.5));
}

void CollisionApp::drawGui() {
    ImGui::Begin("Main Menu");

    if (ImGui::TreeNode("Primitives")) {
        for (auto& primitive : primitives)
            primitive->drawGui("0");
        ImGui::TreePop();
    }

    ImGui::EnumSelection<PRIMITIVES>("Primitive - A", primitiveIndices[A]);
    ImGui::EnumSelection<PRIMITIVES>("Primitive - B", primitiveIndices[B]);

    ImGui::End();
}

void CollisionApp::drawGuizmo() {
    if (selectedRBState) {
        tools::Transformation trafo = parent->rigidBody.getTransformationFromState(*selectedRBState);
        static Eigen::Vector3d scale = Eigen::Vector3d::Ones();
        gui::Guizmo::useWidget(trafo.position, trafo.orientation, scale);
        *selectedRBState = parent->rigidBody.getStateFromTransformation(trafo);
    }
}

void CollisionApp::mouseButtonCallback(double xPos, double yPos, Ray ray, int button, int action) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        static const gui::Model sphere(LENNY_GUI_OPENGL_FOLDER "/data/meshes/sphere.obj");

        selectedRBState = nullptr;
        for (Eigen::VectorXd& rbState : rbStates) {
            const tools::Transformation trafo = parent->rigidBody.getTransformationFromState(rbState);
            const auto hitInfo = sphere.hitByRay(trafo.position, trafo.orientation, Eigen::Vector3d::Ones(), ray);
            if (hitInfo.has_value()) {
                selectedRBState = &rbState;
                break;
            }
        }
    }
}

}  // namespace lenny