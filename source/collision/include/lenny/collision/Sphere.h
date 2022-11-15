#pragma once

#include <lenny/collision/Primitive.h>

namespace lenny::collision {

class Sphere : public Primitive {
public:
    //--- Constructor
    Sphere(const Parent::SPtr parent, const Eigen::Vector3d& position, const double& radius);
    ~Sphere() = default;

    //--- Helpers
    void set(const Eigen::Vector3d& position, const double& radius);
    void get(Eigen::Vector3d& position, double& radius) const;

    //--- Drawing
    void drawScene(const Eigen::VectorXd& parentState, const Eigen::Vector4d& color) const override;
    void drawGui(const std::string& description) override;

    //--- Save and load
    void to_json(json& j) const override;
    void from_json(const json& j) override;
};

}  // namespace lenny::collision