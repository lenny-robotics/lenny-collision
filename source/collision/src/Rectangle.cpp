#include <lenny/collision/Rectangle.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Logger.h>
#include <lenny/tools/Renderer.h>

namespace lenny::collision {

Rectangle::Rectangle(const Parent::SPtr parent, const Eigen::Vector3d& center, const Eigen::QuaternionD& orientation, const Eigen::Vector2d& dimensions,
                     const double& safetyMargin)
    : Primitive("Rectangle", parent) {
    set(center, orientation, dimensions, safetyMargin);
}

void Rectangle::set(const Eigen::Vector3d& center, const Eigen::QuaternionD& orientation, const Eigen::Vector2d& dimensions, const double& safetyMargin) {
    if (dimensions[0] <= 0.0 || dimensions[1] <= 0.0)
        LENNY_LOG_ERROR("Invalid input dimension")

    point = center - orientation * Eigen::Vector3d(dimensions[0], 0.0, dimensions[1]) * 0.5;
    vectors = {orientation.matrix().col(0) * dimensions[0], orientation.matrix().col(2) * dimensions[1]};
    this->safetyMargin = safetyMargin;
    if (this->safetyMargin < 0.0)
        this->safetyMargin = 0.0;
}

void Rectangle::get(Eigen::Vector3d& center, Eigen::QuaternionD& orientation, Eigen::Vector2d& dimensions, double& safetyMargin) const {
    Eigen::Matrix3d orientationMatrix;
    orientationMatrix.col(0) = vectors.at(0).normalized();
    orientationMatrix.col(2) = vectors.at(1).normalized();
    orientationMatrix.col(1) = orientationMatrix.col(0).cross(orientationMatrix.col(2));
    if ((orientationMatrix.determinant() + 1.0) < 1e-5)
        orientationMatrix.col(1) *= -1.0;
    orientation = Eigen::QuaternionD(orientationMatrix);
    dimensions[0] = vectors.at(0).norm();
    dimensions[1] = vectors.at(1).norm();
    center = point + orientation * Eigen::Vector3d(dimensions[0], 0.0, dimensions[1]) * 0.5;
    safetyMargin = this->safetyMargin;
}

void Rectangle::drawScene(const Eigen::VectorXd& parentState, const Eigen::Vector4d& color) const {
    static Eigen::Vector3d center;
    static Eigen::QuaternionD orientation;
    static Eigen::Vector2d dimensions;
    static double safetyMargin;
    get(center, orientation, dimensions, safetyMargin);

    const Eigen::Vector3d P_c = parent->computePoint(parentState, center);
    Eigen::Matrix3d R_c;
    for (int i = 0; i < 3; i++)
        R_c.col(i) = parent->computeVector(parentState, orientation.matrix().col(i));

    tools::Renderer::I->drawPlane(P_c, Eigen::QuaternionD(R_c), dimensions, color);
}

void Rectangle::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(("Rectangle - " + description).c_str())) {
        static Eigen::Vector3d center;
        static Eigen::QuaternionD orientation;
        static Eigen::Vector2d dimensions;
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

void Rectangle::to_json(json& j) const {
    Primitive::to_json(j);
    static Eigen::Vector3d center;
    static Eigen::QuaternionD orientation;
    static Eigen::Vector2d dimensions;
    static double safetyMargin;
    get(center, orientation, dimensions, safetyMargin);
    j["center"] = center;
    j["orientation"] = orientation;
    j["dimensions"] = dimensions;
    j["safetyMargin"] = safetyMargin;
}

void Rectangle::from_json(const json& j) {
    Primitive::from_json(j);
    static Eigen::Vector3d center;
    static Eigen::QuaternionD orientation;
    static Eigen::Vector2d dimensions;
    static double safetyMargin;
    center = j.value("center", Eigen::Vector3d());
    orientation = j.value("orientation", Eigen::QuaternionD());
    dimensions = j.value("dimensions", Eigen::Vector2d());
    safetyMargin = j.value("safetyMargin", double());
    set(center, orientation, dimensions, safetyMargin);
}

}  // namespace lenny::collision