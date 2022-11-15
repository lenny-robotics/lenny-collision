#pragma once

#include <lenny/collision/Primitive.h>

namespace lenny::collision {

class Rectangle : public Primitive {
public:
    //--- Constructor
    Rectangle(const Parent::SPtr parent, const Eigen::Vector3d& center, const Eigen::QuaternionD& orientation, const Eigen::Vector2d& dimensions,
              const double& safetyMargin);
    ~Rectangle() = default;

    //--- Helpers
    void set(const Eigen::Vector3d& center, const Eigen::QuaternionD& orientation, const Eigen::Vector2d& dimensions, const double& safetyMargin);
    void get(Eigen::Vector3d& center, Eigen::QuaternionD& orientation, Eigen::Vector2d& dimensions, double& safetyMargin) const;

    //--- Drawing
    void drawScene(const Eigen::VectorXd& parentState, const Eigen::Vector4d& color) const override;
    void drawGui(const std::string& description) override;

    //--- Save and load
    void to_json(json& j) const override;
    void from_json(const json& j) override;
};

}  // namespace lenny::collision