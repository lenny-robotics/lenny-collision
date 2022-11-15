#pragma once

#include <lenny/collision/Primitive.h>

namespace lenny::collision {

class Capsule : public Primitive {
public:
    //--- Constructor
    Capsule(const Parent::SPtr parent, const Eigen::Vector3d& startPosition, const Eigen::Vector3d& endPosition, const double& radius);
    ~Capsule() = default;

    //--- Helpers
    void set(const Eigen::Vector3d& startPosition, const Eigen::Vector3d& endPosition, const double& radius);
    void get(Eigen::Vector3d& startPosition, Eigen::Vector3d& endPosition, double& radius) const;

    //--- Drawing
    void drawScene(const Eigen::VectorXd& parentState, const Eigen::Vector4d& color) const override;
    void drawGui(const std::string& description) override;

    //--- Save and load
    void to_json(json& j) const override;
    void from_json(const json& j) override;
};

}  // namespace lenny::collision