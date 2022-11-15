#pragma once

#include <lenny/collision/Primitive.h>

namespace lenny::collision {

/*
 * COMMENT: s and t are both stacked vectors of the parameters belonging to both primitives
 * -> s = [s_first, s_second] (state of primitive parents)
 * -> t = [t_first, t_second] (parameters determining where closest point is)
 */

class DistanceCalculator {
public:
    //--- Constructor
    DistanceCalculator(const Primitive::SPtr primitive_A, const Primitive::SPtr primitive_B);
    ~DistanceCalculator() = default;

    //--- Computation
    double compute_D(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void compute_pDpS(Eigen::VectorXd& pDpS, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void compute_p2DpS2(Eigen::MatrixXd& p2DpS2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void compute_pDpT(Eigen::VectorXd& pDpT, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void compute_p2DpT2(Eigen::MatrixXd& p2DpT2, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void compute_p2DpTpS(Eigen::MatrixXd& p2DpTpS, const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;

    //--- Tests
    void test_pDpS(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void test_p2DpS2(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void test_pDpT(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void test_p2DpT2(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;
    void test_p2DpTpS(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;

    //--- Helpers
    std::pair<Eigen::Vector3d, Eigen::Vector3d> compute_Ps(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;

    std::pair<Eigen::VectorXd, Eigen::VectorXd> getTs(const Eigen::VectorXd& t) const;
    std::pair<int, int> getSizesOfT() const;
    int getTotalSizeOfT() const;

    std::pair<Eigen::VectorXd, Eigen::VectorXd> getSs(const Eigen::VectorXd& s) const;
    std::pair<int, int> getSizesOfS() const;
    int getTotalSizeOfS() const;

    void checkInputs(const Eigen::VectorXd& s, const Eigen::VectorXd& t) const;

public:
    const Primitive::SPtr primitive_A;
    const Primitive::SPtr primitive_B;

private:
    static tools::FiniteDifference fd;
};

}  // namespace lenny::collision