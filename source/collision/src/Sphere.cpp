#include <lenny/collision/Sphere.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Renderer.h>

namespace lenny::collision {

Sphere::Sphere(const Parent::SPtr parent, const Eigen::Vector3d& position, const double& radius) : Primitive("Sphere", parent) {
    set(position, radius);
}

void Sphere::set(const Eigen::Vector3d& position, const double& radius) {
    point = position;
    vectors.clear();
    safetyMargin = radius;
    if (safetyMargin < 0.0)
        safetyMargin = 0.0;
}

void Sphere::get(Eigen::Vector3d& position, double& radius) const {
    position = point;
    radius = safetyMargin;
}

void Sphere::drawScene(const Eigen::VectorXd& parentState, const Eigen::Vector4d& color) const {
    static Eigen::Vector3d position;
    static double radius;
    get(position, radius);
    tools::Renderer::I->drawSphere(parent->computePoint(parentState, position), radius, color);
}

void Sphere::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(("Sphere - " + description).c_str())) {
        static Eigen::Vector3d position;
        static double radius;
        get(position, radius);

        if (Gui::I->Input("Position", position))
            set(position, radius);
        if (Gui::I->Input("Radius", radius))
            set(position, radius);

        if (Gui::I->Button("Print Infos"))
            printInfos();

        Gui::I->TreePop();
    }
}

void Sphere::to_json(json& j) const {
    Primitive::to_json(j);
    static Eigen::Vector3d position;
    static double radius;
    get(position, radius);
    j["position"] = position;
    j["radius"] = radius;
}

void Sphere::from_json(const json& j) {
    Primitive::from_json(j);
    static Eigen::Vector3d position;
    static double radius;
    position = j.value("position", Eigen::Vector3d());
    radius = j.value("radius", double());
    set(position, radius);
}

}  // namespace lenny::collision