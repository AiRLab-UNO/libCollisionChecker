#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <filesystem>
#include <Eigen/Dense>
#include "CollisionChecker.hh"
#include "common/ParamManager.h"

namespace fs = std::filesystem;

class FCLCollisionCheckerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ParamManager with some default parameters
        params_ = std::make_shared<param_manager>();
        params_->add_param("robot_radius", 0.5f);
        params_->add_param("obstacle_length", 1.0f);
        
        std::vector<float> boundary = {-10.0f, 10.0f, -10.0f, 10.0f};
        params_->add_param("boundary", boundary);
        
        // Add one obstacle at (5, 5, 0)
        std::vector<std::vector<float>> obstacles = {{5.0f, 5.0f, 0.0f}};
        params_->add_ndarray("obstacles", obstacles);
    }

    std::vector<Eigen::VectorXd> createTrajectory(const std::vector<std::vector<double>>& points) {
        std::vector<Eigen::VectorXd> trajectory;
        for (const auto& point : points) {
            Eigen::VectorXd state(point.size());
            for (size_t i = 0; i < point.size(); ++i) {
                state(i) = point[i];
            }
            trajectory.push_back(state);
        }
        return trajectory;
    }

    std::shared_ptr<param_manager> params_;
};

// Basic test to verify FCL collision checker can be instantiated
TEST_F(FCLCollisionCheckerTest, BasicInstantiation) {
    auto checker = std::make_shared<fcl::FCL>(params_);
    EXPECT_NE(checker, nullptr);
}

// Test: Empty trajectory
TEST_F(FCLCollisionCheckerTest, EmptyTrajectory) {
    auto checker = std::make_shared<fcl::FCL>(params_);
    std::vector<Eigen::VectorXd> trajectory;
    EXPECT_FALSE(checker->isCollision(trajectory));
}

// Test: Single point trajectory - No collision
TEST_F(FCLCollisionCheckerTest, SinglePointNoCollision) {
    auto checker = std::make_shared<fcl::FCL>(params_);
    auto trajectory = createTrajectory({{0.0, 0.0, 0.0}});
    EXPECT_FALSE(checker->isCollision(trajectory));
}

// Test: Single point trajectory - Collision
TEST_F(FCLCollisionCheckerTest, SinglePointCollision) {
    auto checker = std::make_shared<fcl::FCL>(params_);
    // Obstacle is at (5, 5, 0) with half-length 1.0 (total 2.0).
    // Box is from (4, 4, -1) to (6, 6, 1).
    // Robot radius is 0.5. At (5, 5, 0), it should collide.
    auto trajectory = createTrajectory({{5.0, 5.0, 0.0}});
    EXPECT_TRUE(checker->isCollision(trajectory));
}

// Test: Boundary violation
TEST_F(FCLCollisionCheckerTest, BoundaryViolation) {
    auto checker = std::make_shared<fcl::FCL>(params_);
    auto trajectory = createTrajectory({{11.0, 0.0, 0.0}});
    EXPECT_TRUE(checker->isCollision(trajectory));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
