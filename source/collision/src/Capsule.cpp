#include <lenny/collision/Capsule.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Renderer.h>

namespace lenny::collision {

Capsule::Capsule(const Parent::SPtr parent, const Eigen::Vector3d& startPosition, const Eigen::Vector3d& endPosition, const double& radius)
    : Primitive("Capsule", parent) {
    set(startPosition, endPosition, radius);
}

void Capsule::set(const Eigen::Vector3d& startPosition, const Eigen::Vector3d& endPosition, const double& radius) {
    point = startPosition;
    vectors = {endPosition - startPosition};
    safetyMargin = radius;
    if (safetyMargin < 0.0)
        safetyMargin = 0.0;
}

void Capsule::get(Eigen::Vector3d& startPosition, Eigen::Vector3d& endPosition, double& radius) const {
    startPosition = point;
    endPosition = startPosition + vectors.at(0);
    radius = safetyMargin;
}

void Capsule::drawScene(const Eigen::VectorXd& parentState, const Eigen::Vector4d& color) const {
    static Eigen::Vector3d startPosition, endPosition;
    static double radius;
    get(startPosition, endPosition, radius);
    const Eigen::Vector3d P1 = parent->computePoint(parentState, startPosition);
    const Eigen::Vector3d P2 = parent->computePoint(parentState, endPosition);
    tools::Renderer::I->drawCapsule(P1, P2, radius, color);
}

void Capsule::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(("Capsule - " + description).c_str())) {
        static Eigen::Vector3d startPosition, endPosition;
        static double radius;
        get(startPosition, endPosition, radius);

        if (Gui::I->Input("Start Position", startPosition))
            set(startPosition, endPosition, radius);
        if (Gui::I->Input("End Position", endPosition))
            set(startPosition, endPosition, radius);
        if (Gui::I->Input("Radius", radius))
            set(startPosition, endPosition, radius);

        if (Gui::I->Button("Print Infos"))
            printInfos();

        Gui::I->TreePop();
    }
}

void Capsule::to_json(json& j) const {
    Primitive::to_json(j);
    static Eigen::Vector3d startPosition, endPosition;
    static double radius;
    get(startPosition, endPosition, radius);
    j["startPosition"] = startPosition;
    j["endPosition"] = endPosition;
    j["radius"] = radius;
}

void Capsule::from_json(const json& j) {
    Primitive::from_json(j);
    static Eigen::Vector3d startPosition, endPosition;
    static double radius;
    startPosition = j.value("startPosition", Eigen::Vector3d());
    endPosition = j.value("endPosition", Eigen::Vector3d());
    radius = j.value("radius", double());
    set(startPosition, endPosition, radius);
}

}  // namespace lenny::collision