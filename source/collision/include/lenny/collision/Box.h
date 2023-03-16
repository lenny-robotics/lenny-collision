#pragma once

#include <lenny/collision/Primitive.h>

namespace lenny::collision {

class Box : public Primitive {
public:
    //--- Constructor
    Box(const Parent::SPtr parent, const Eigen::Vector3d& center, const Eigen::QuaternionD& orientation, const Eigen::Vector3d& dimensions,
        const double& safetyMargin);
    ~Box() = default;

    //--- Helpers
    void set(const Eigen::Vector3d& center, const Eigen::QuaternionD& orientation, const Eigen::Vector3d& dimensions, const double& safetyMargin);
    void get(Eigen::Vector3d& center, Eigen::QuaternionD& orientation, Eigen::Vector3d& dimensions, double& safetyMargin) const;
    double getSafetyMargin(const double& D) const override;

    //--- Drawing
    void drawScene(const Eigen::VectorXd& parentState, const Eigen::Vector4d& color) const override;
    void drawGui(const std::string& description) override;

    //--- Save and load
    void to_json(json& j) const override;
    void from_json(const json& j) override;
};

}  // namespace lenny::collision