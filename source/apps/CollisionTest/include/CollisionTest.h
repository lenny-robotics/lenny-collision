#pragma once

#include <lenny/collision/Api.h>
#include <lenny/collision/Primitives.h>
#include <lenny/collision/Solver.h>
#include <lenny/tools/Utils.h>

#include <iostream>

#include "EulerAngleRigidBodyParent.h"

namespace lenny {

void applyCollisionTest() {
    //--- Initialize
    EulerAngleRigidBodyParent::SPtr parent = std::make_shared<EulerAngleRigidBodyParent>();
    tools::FiniteDifference fd("Estimation Tester");

    Eigen::VectorXd rbState(6);
    for (int i = 0; i < 6; i++)
        rbState[i] = tools::utils::getRandomNumberInRange({-PI, PI});

    auto generatePrimitives = [&]() -> std::vector<collision::Primitive::SPtr> {
        return {
            std::make_shared<collision::Sphere>(parent, Eigen::Vector3d::Random(), tools::utils::getRandomNumberInRange({0.1, 1.0})),
            std::make_shared<collision::Capsule>(parent, Eigen::Vector3d::Random(), Eigen::Vector3d::Random(),
                                                 tools::utils::getRandomNumberInRange({0.1, 1.0})),
            std::make_shared<collision::Rectangle>(
                parent, Eigen::Vector3d::Random(), Eigen::QuaternionD::UnitRandom(),
                Eigen::Vector2d(tools::utils::getRandomNumberInRange({0.1, 1.0}), tools::utils::getRandomNumberInRange({0.1, 1.0})),
                tools::utils::getRandomNumberInRange({0.1, 1.0})),
            std::make_shared<collision::Box>(parent, Eigen::Vector3d::Random(), Eigen::QuaternionD::UnitRandom(),
                                             Eigen::Vector3d(tools::utils::getRandomNumberInRange({0.1, 1.0}), tools::utils::getRandomNumberInRange({0.1, 1.0}),
                                                             tools::utils::getRandomNumberInRange({0.1, 1.0})),
                                             tools::utils::getRandomNumberInRange({0.1, 1.0}))};
    };

    std::vector<collision::Primitive::SPtr> primitives = generatePrimitives();

    //--- Test parent
    {
        //--- Point
        const Eigen::Vector3d p_local = Eigen::Vector3d::Random();
        std::cout << parent->computePoint(rbState, p_local).transpose() << std::endl;
        parent->testPointJacobian(rbState, p_local);
        parent->testPointTensor(rbState, p_local);

        Eigen::MatrixXd p_jac_est, p_jac_ana;
        parent->estimatePointJacobian(p_jac_est, rbState, p_local);
        parent->computePointJacobian(p_jac_ana, rbState, p_local);
        fd.performCheck(p_jac_est, p_jac_ana, "Point Jacobian");

        Eigen::TensorD p_ten_est, p_ten_ana;
        parent->estimatePointTensor(p_ten_est, rbState, p_local);
        parent->computePointTensor(p_ten_ana, rbState, p_local);
        fd.performCheck(p_ten_est, p_ten_ana, "Point Tensor");

        //--- Vector
        const Eigen::Vector3d v_local = Eigen::Vector3d::Random();
        std::cout << parent->computeVector(rbState, v_local).transpose() << std::endl;
        parent->testVectorJacobian(rbState, v_local);
        parent->testVectorTensor(rbState, v_local);

        Eigen::MatrixXd v_jac_est, v_jac_ana;
        parent->estimateVectorJacobian(v_jac_est, rbState, v_local);
        parent->computeVectorJacobian(v_jac_ana, rbState, v_local);
        fd.performCheck(v_jac_est, v_jac_ana, "Vector Jacobian");

        Eigen::TensorD v_ten_est, v_ten_ana;
        parent->estimateVectorTensor(v_ten_est, rbState, v_local);
        parent->computeVectorTensor(v_ten_ana, rbState, v_local);
        fd.performCheck(v_ten_est, v_ten_ana, "Vector Tensor");
    }

    //--- Test primitives
    {
        for (const auto& primitive : primitives) {
            const Eigen::VectorXd t = Eigen::VectorXd::Random(primitive->getSizeOfT());

            //--- Derivatives
            std::cout << primitive->compute_P(rbState, t).transpose() << std::endl;
            primitive->test_pPpT(rbState, t);
            primitive->test_p2PpT2(rbState, t);
            primitive->test_pPpS(rbState, t);
            primitive->test_p2PpS2(rbState, t);
            primitive->test_p2PpSpT(rbState, t);

            //--- Estimates
            Eigen::MatrixXd pPpS_est, pPpS_ana;
            primitive->estimate_pPpS(pPpS_est, rbState, t);
            primitive->compute_pPpS(pPpS_ana, rbState, t);
            fd.performCheck(pPpS_est, pPpS_ana, "pPpS");

            Eigen::TensorD p2PpS2_est, p2PpS2_ana;
            primitive->estimate_p2PpS2(p2PpS2_est, rbState, t);
            primitive->compute_p2PpS2(p2PpS2_ana, rbState, t);
            fd.performCheck(p2PpS2_est, p2PpS2_ana, "p2PpS2");

            Eigen::MatrixXd pPpT_est, pPpT_ana;
            primitive->estimate_pPpT(pPpT_est, rbState, t);
            primitive->compute_pPpT(pPpT_ana, rbState, t);
            fd.performCheck(pPpT_est, pPpT_ana, "pPpT");

            Eigen::TensorD p2PpT2_est, p2PpT2_ana;
            primitive->estimate_p2PpT2(p2PpT2_est, rbState, t);
            primitive->compute_p2PpT2(p2PpT2_ana, rbState, t);
            fd.performCheck(p2PpT2_est, p2PpT2_ana, "p2PpT2");

            Eigen::TensorD p2PpSpT_est, p2PpSpT_ana;
            primitive->estimate_p2PpSpT(p2PpSpT_est, rbState, t);
            primitive->compute_p2PpSpT(p2PpSpT_ana, rbState, t);
            fd.performCheck(p2PpSpT_est, p2PpSpT_ana, "p2PpSpT");
        }

        for (const auto& primitive : primitives) {
            //--- Getter & Setter
            if (auto p = std::dynamic_pointer_cast<collision::Sphere>(primitive)) {
                const Eigen::Vector3d position = Eigen::Vector3d::Random();
                const double radius = tools::utils::getRandomNumberInRange({0.0, 1.0});
                p->set(position, radius);

                Eigen::Vector3d testPosition;
                double testRadius;
                p->get(testPosition, testRadius);

                if (position.isApprox(testPosition) && fabs(radius - testRadius) < 1e-5)
                    LENNY_LOG_PRINT(tools::Logger::GREEN, "Sphere getter / setter test PASSED\n")
                else
                    LENNY_LOG_PRINT(tools::Logger::RED, "Sphere getter / setter test FAILED\n")
            } else if (auto p = std::dynamic_pointer_cast<collision::Capsule>(primitive)) {
                const Eigen::Vector3d startPosition = Eigen::Vector3d::Random();
                const Eigen::Vector3d endPosition = Eigen::Vector3d::Random();
                const double radius = tools::utils::getRandomNumberInRange({0.0, 1.0});
                p->set(startPosition, endPosition, radius);

                Eigen::Vector3d testStartPosition, testEndPosition;
                double testRadius;
                p->get(testStartPosition, testEndPosition, testRadius);

                if (startPosition.isApprox(testStartPosition) && endPosition.isApprox(testEndPosition) && fabs(radius - testRadius) < 1e-5)
                    LENNY_LOG_PRINT(tools::Logger::GREEN, "Capsule getter / setter test PASSED\n")
                else
                    LENNY_LOG_PRINT(tools::Logger::RED, "Capsule getter / setter test FAILED\n")
            } else if (auto p = std::dynamic_pointer_cast<collision::Rectangle>(primitive)) {
                const Eigen::Vector3d center = Eigen::Vector3d::Random();
                const Eigen::QuaternionD orientation = Eigen::QuaternionD ::UnitRandom();
                const Eigen::Vector2d dimension =
                    Eigen::Vector2d(tools::utils::getRandomNumberInRange({0.1, 1.0}), tools::utils::getRandomNumberInRange({0.1, 1.0}));
                const double safetyMargin = tools::utils::getRandomNumberInRange({0.0, 1.0});
                p->set(center, orientation, dimension, safetyMargin);

                Eigen::Vector3d testCenter;
                Eigen::QuaternionD testOrientation;
                Eigen::Vector2d testDimension;
                double testSafetyMargin;
                p->get(testCenter, testOrientation, testDimension, testSafetyMargin);

                if (center.isApprox(testCenter) && orientation.matrix().isApprox(testOrientation.matrix()) && dimension.isApprox(testDimension) &&
                    fabs(safetyMargin - testSafetyMargin) < 1e-5)
                    LENNY_LOG_PRINT(tools::Logger::GREEN, "Rectangle getter / setter test PASSED\n")
                else
                    LENNY_LOG_PRINT(tools::Logger::RED, "Rectangle getter / setter test FAILED\n")
            } else if (auto p = std::dynamic_pointer_cast<collision::Box>(primitive)) {
                const Eigen::Vector3d center = Eigen::Vector3d::Random();
                const Eigen::QuaternionD orientation = Eigen::QuaternionD ::UnitRandom();
                const Eigen::Vector3d dimension =
                    Eigen::Vector3d(tools::utils::getRandomNumberInRange({0.1, 1.0}), tools::utils::getRandomNumberInRange({0.1, 1.0}),
                                    tools::utils::getRandomNumberInRange({0.1, 1.0}));
                const double safetyMargin = tools::utils::getRandomNumberInRange({0.0, 1.0});
                p->set(center, orientation, dimension, safetyMargin);

                Eigen::Vector3d testCenter;
                Eigen::QuaternionD testOrientation;
                Eigen::Vector3d testDimension;
                double testSafetyMargin;
                p->get(testCenter, testOrientation, testDimension, testSafetyMargin);

                if (center.isApprox(testCenter) && orientation.matrix().isApprox(testOrientation.matrix()) && dimension.isApprox(testDimension) &&
                    fabs(safetyMargin - testSafetyMargin) < 1e-5)
                    LENNY_LOG_PRINT(tools::Logger::GREEN, "Box getter / setter test PASSED\n")
                else
                    LENNY_LOG_PRINT(tools::Logger::RED, "Box getter / setter test FAILED\n")
            }
        }
    }

    //--- Test pair computations
    {
        std::vector<collision::Primitive::SPtr> secondPrimList = generatePrimitives();
        Eigen::VectorXd s(2 * rbState.size());
        s << rbState, rbState;

        //--- Distance Calculator
        for (int i = 0; i < primitives.size(); i++) {
            for (int j = 0; j < secondPrimList.size(); j++) {
                collision::DistanceCalculator distanceCalculator(primitives[i], secondPrimList[j]);
                const Eigen::VectorXd t = Eigen::VectorXd::Random(distanceCalculator.getTotalSizeOfT());

                std::cout << distanceCalculator.compute_D(s, t) << std::endl;
                distanceCalculator.test_pDpS(s, t);
                distanceCalculator.test_p2DpS2(s, t);
                distanceCalculator.test_pDpT(s, t);
                distanceCalculator.test_p2DpT2(s, t);
                distanceCalculator.test_p2DpTpS(s, t);
            }
        }

        //--- Objective
        for (int i = 0; i < primitives.size(); i++) {
            for (int j = 0; j < secondPrimList.size(); j++) {
                collision::TotalObjective objective(primitives[i], secondPrimList[j], s);
                const Eigen::VectorXd t = Eigen::VectorXd::Random(objective.distanceCalculator.getTotalSizeOfT());

                std::cout << objective.computeValue(t) << std::endl;
                objective.testGradient(t);
                objective.testHessian(t);
            }
        }

        //--- Solver
        for (int i = 0; i < primitives.size(); i++) {
            for (int j = 0; j < secondPrimList.size(); j++) {
                collision::Solver solver(primitives[i], secondPrimList[j], s);
                solver.optimizer.printInfos = true;

                Eigen::VectorXd t = 0.5 * Eigen::VectorXd::Ones(solver.objective.distanceCalculator.getTotalSizeOfT());
                solver.compute_T(t);
                std::cout << t.transpose() << std::endl;
                solver.test_dTdS(t);
                solver.test_dDdS(t);
                solver.test_d2DdS2(t);
            }
        }

        //--- Api
        for (int i = 0; i < primitives.size(); i++) {
            for (int j = 0; j < secondPrimList.size(); j++) {
                const collision::Api::PrimitiveInfo pi_a = {primitives[i], rbState};
                const collision::Api::PrimitiveInfo pi_b = {secondPrimList[i], rbState};

                Eigen::VectorXd t;
                collision::Api::compute_T(t, pi_a, pi_b);

                const auto [P1, P2] = collision::Api::computeClosestPoints(t, pi_a, pi_b);
                std::cout << P1.transpose() << "   /   " << P2.transpose() << std::endl;

                std::cout << collision::Api::compute_D(t, pi_a, pi_b) << std::endl;

                Eigen::VectorXd dDdS;
                collision::Api::compute_dDdS(dDdS, t, pi_a, pi_b);
                std::cout << dDdS.transpose() << std::endl;

                Eigen::MatrixXd d2DdS2;
                collision::Api::compute_d2DdS2(d2DdS2, t, pi_a, pi_b);
                std::cout << d2DdS2 << std::endl;

                std::cout << "------------------------------------------------------------------------" << std::endl;
            }
        }
    }

    //--- Save & load
    collision::savePrimitivesToFile(primitives, LENNY_PROJECT_FOLDER "/logs/Primitives.json");
    std::vector<collision::Primitive::SPtr> loadPrimitives;
    collision::F_getParent getParent = [&](const std::string&) -> const collision::Parent::SPtr { return parent; };
    collision::loadPrimitivesFromFile(loadPrimitives, getParent, LENNY_PROJECT_FOLDER "/logs/Primitives.json");
    json js;
    collision::to_json(js, loadPrimitives);
    std::cout << std::setw(2) << js << std::endl;
}

}  // namespace lenny