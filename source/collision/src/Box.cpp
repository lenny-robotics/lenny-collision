#include <lenny/collision/Box.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Logger.h>
#include <lenny/tools/Renderer.h>

namespace lenny::collision {

Box::Box(const Parent::SPtr parent, const Eigen::Vector3d& center, const Eigen::QuaternionD& orientation, const Eigen::Vector3d& dimensions,
         const double& safetyMargin)
    : Primitive("Box", parent) {
    set(center, orientation, dimensions, safetyMargin);
}

void Box::set(const Eigen::Vector3d& center, const Eigen::QuaternionD& orientation, const Eigen::Vector3d& dimensions, const double& safetyMargin) {
    if (dimensions[0] <= 0.0 || dimensions[1] <= 0.0 || dimensions[2] <= 0.0)
        LENNY_LOG_ERROR("Invalid input dimension")

    point = center - orientation * dimensions * 0.5;
    vectors.clear();
    for (int i = 0; i < 3; i++)
        vectors.push_back(orientation.matrix().col(i) * dimensions[i]);
    this->safetyMargin = safetyMargin;
    if (this->safetyMargin < 0.0)
        this->safetyMargin = 0.0;
}

void Box::get(Eigen::Vector3d& center, Eigen::QuaternionD& orientation, Eigen::Vector3d& dimensions, double& safetyMargin) const {
    Eigen::Matrix3d orientationMatrix;
    for (int i = 0; i < 3; i++) {
        orientationMatrix.col(i) = vectors.at(i).normalized();
        dimensions[i] = vectors.at(i).norm();
    }
    orientation = Eigen::QuaternionD(orientationMatrix);
    center = point + orientation * dimensions * 0.5;
    safetyMargin = this->safetyMargin;
}

double Box::getSafetyMargin(const double& D) const {
    double margin = this->safetyMargin;
    if (D < 1e-3)
        margin += 0.5 * std::min(std::min(vectors.at(0).norm(), vectors.at(1).norm()), vectors.at(2).norm());
    return margin;
}

void Box::drawScene(const Eigen::VectorXd& parentState, const Eigen::Vector4d& color) const {
    static Eigen::Vector3d center;
    static Eigen::QuaternionD orientation;
    static Eigen::Vector3d dimensions;
    static double safetyMargin;
    get(center, orientation, dimensions, safetyMargin);

    const Eigen::Vector3d P_c = parent->computePoint(parentState, center);
    Eigen::Matrix3d R_c;
    for (int i = 0; i < 3; i++)
        R_c.col(i) = parent->computeVector(parentState, orientation.matrix().col(i));

    tools::Renderer::I->drawRoundedCuboid(P_c, Eigen::QuaternionD(R_c), dimensions, safetyMargin, color);
}

void Box::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(("Box - " + description).c_str())) {
        static Eigen::Vector3d center;
        static Eigen::QuaternionD orientation;
        static Eigen::Vector3d dimensions;
        static double safetyMargin;
        get(center, orientation, dimensions, safetyMargin);

        if (Gui::I->Input("Center", center))
            set(center, orientation, dimensions, safetyMargin);
        if (Gui::I->Slider("Orientation", orientation))
            set(center, orientation, dimensions, safetyMargin);
        if (Gui::I->Input("Dimensions", dimensions))
            set(center, orientation, dimensions, safetyMargin);
        if (Gui::I->Input("Safety Margin", safetyMargin))
            set(center, orientation, dimensions, safetyMargin);

        if (Gui::I->Button("Print Infos"))
            printInfos();

        Gui::I->TreePop();
    }
}

void Box::to_json(json& j) const {
    Primitive::to_json(j);
    static Eigen::Vector3d center;
    static Eigen::QuaternionD orientation;
    static Eigen::Vector3d dimensions;
    static double safetyMargin;
    get(center, orientation, dimensions, safetyMargin);
    j["center"] = center;
    j["orientation"] = orientation;
    j["dimensions"] = dimensions;
    j["safetyMargin"] = safetyMargin;
}

void Box::from_json(const json& j) {
    Primitive::from_json(j);
    static Eigen::Vector3d center;
    static Eigen::QuaternionD orientation;
    static Eigen::Vector3d dimensions;
    static double safetyMargin;
    center = j.value("center", Eigen::Vector3d());
    orientation = j.value("orientation", Eigen::QuaternionD());
    dimensions = j.value("dimensions", Eigen::Vector3d());
    safetyMargin = j.value("safetyMargin", double());
    set(center, orientation, dimensions, safetyMargin);
}

}  // namespace lenny::collision