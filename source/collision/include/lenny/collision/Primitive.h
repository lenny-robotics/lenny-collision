#pragma once

#include <lenny/collision/Parent.h>
#include <lenny/tools/Json.h>

namespace lenny::collision {

class Primitive {
public:
    //--- Pointers
    typedef std::shared_ptr<Primitive> SPtr;
    typedef std::shared_ptr<const Primitive> CSPtr;

    //--- Constructor
    Primitive(const std::string& description, const Parent::SPtr parent);
    virtual ~Primitive() = default;

    //--- Compute point and its partial derivatives based on state (S) and parameters (T)
    Eigen::Vector3d compute_P(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void compute_pPpS(Eigen::MatrixXd& pPpS, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void compute_p2PpS2(Eigen::TensorD& p2PpS2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void compute_pPpT(Eigen::MatrixXd& pPpT, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void compute_p2PpT2(Eigen::TensorD& p2PpT2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void compute_p2PpSpT(Eigen::TensorD& p2PpSpT, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;

    //--- Estimations
    void estimate_pPpS(Eigen::MatrixXd& pPpS, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void estimate_p2PpS2(Eigen::TensorD& p2PpS2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void estimate_pPpT(Eigen::MatrixXd& pPpT, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void estimate_p2PpT2(Eigen::TensorD& p2PpT2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void estimate_p2PpSpT(Eigen::TensorD& p2PpSpT, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;

    //--- Tests
    bool test_pPpS(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    bool test_p2PpS2(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    bool test_pPpT(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    bool test_p2PpT2(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    bool test_p2PpSpT(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;

    //--- Helpers
    int getSizeOfS() const;
    int getSizeOfT() const;
    void printInfos() const;
    virtual double getSafetyMargin(const double& D) const;

    //--- Drawing
    virtual void drawScene(const Eigen::VectorXd& parentState, const Eigen::Vector4d& color) const = 0;
    virtual void drawGui(const std::string& description) = 0;

    //--- Save and load
    virtual void to_json(json& j) const;
    virtual void from_json(const json& j);

protected:
    //--- Helpers
    void checkInputs(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;

public:
    std::string description;    //Set by constructor
    const Parent::SPtr parent;  //Set by constructor

protected:
    double safetyMargin = 0.0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> vectors = {};

    static tools::FiniteDifference fd;
};

}  // namespace lenny::collision