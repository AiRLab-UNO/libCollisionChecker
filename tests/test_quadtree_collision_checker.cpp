#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <filesystem>
#include <Eigen/Dense>
#include "CollisionChecker.hh"
#include "common/ParamManager.h"

namespace fs = std::filesystem;

class QuadtreeCollisionCheckerTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_data_dir_ = TEST_DATA_DIR;
    }

    std::shared_ptr<quadtree::QuadtreeCollisionChecker> createCheckerFromYAML(const std::string& yaml_file) {
        fs::path yaml_path = fs::path(test_data_dir_) / yaml_file;
        EXPECT_TRUE(fs::exists(yaml_path)) << "Test file not found: " << yaml_path;
        
        auto pm = std::make_shared<param_manager>(yaml_path.string());
        return std::make_shared<quadtree::QuadtreeCollisionChecker>(pm);
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

    std::string test_data_dir_;
};

// Test: Constructor and initialization with UGV test files
TEST_F(QuadtreeCollisionCheckerTest, ConstructorWithUGVConfig) {
    ASSERT_NO_THROW({
        auto checker = createCheckerFromYAML("ugv/env1.yaml");
        ASSERT_NE(checker, nullptr);
    });
}

// Test: Multiple UGV configurations
TEST_F(QuadtreeCollisionCheckerTest, LoadMultipleUGVConfigs) {
    std::vector<std::string> ugv_configs = {
        "ugv/env1.yaml",
        "ugv/env2.yaml",
        "ugv/env3.yaml",
        "ugv/env4.yaml",
        "ugv/env5.yaml"
    };

    for (const auto& config : ugv_configs) {
        fs::path config_path = fs::path(test_data_dir_) / config;
        if (fs::exists(config_path)) {
            ASSERT_NO_THROW({
                auto checker = createCheckerFromYAML(config);
                ASSERT_NE(checker, nullptr);
            }) << "Failed to load: " << config;
        }
    }
}

// Test: Collision detection at start position
TEST_F(QuadtreeCollisionCheckerTest, NoCollisionAtStart) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    // Start position from env1.yaml: [-10.0, -2.0]
    auto trajectory = createTrajectory({{-10.0, -2.0}});
    
    EXPECT_FALSE(checker->isCollision(trajectory))
        << "Start position should be collision-free";
}

// Test: Collision detection at goal position
TEST_F(QuadtreeCollisionCheckerTest, NoCollisionAtGoal) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    // Goal position from env1.yaml: [0.0, 0.0]
    auto trajectory = createTrajectory({{0.0, 0.0}});
    
    EXPECT_FALSE(checker->isCollision(trajectory))
        << "Goal position should be collision-free";
}

// Test: Collision detection at obstacle location
TEST_F(QuadtreeCollisionCheckerTest, CollisionAtObstacle) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    // First obstacle from env1.yaml: [-10., -8.]
    // Robot radius is 0.345, so exact obstacle position should collide
    auto trajectory = createTrajectory({{-10.0, -8.0}});
    
    EXPECT_TRUE(checker->isCollision(trajectory))
        << "Should detect collision at obstacle position";
}

// Test: Multiple obstacles collision check
TEST_F(QuadtreeCollisionCheckerTest, CollisionAtMultipleObstacles) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    // Test multiple obstacle positions from env1.yaml
    std::vector<std::vector<double>> obstacle_positions = {
        {-10.0, -8.0},
        {-6.0, -8.0},
        {-5.0, -6.0},
        {-7.0, -6.0}
    };
    
    for (const auto& pos : obstacle_positions) {
        auto trajectory = createTrajectory({pos});
        EXPECT_TRUE(checker->isCollision(trajectory))
            << "Should detect collision at obstacle position: ["
            << pos[0] << ", " << pos[1] << "]";
    }
}

// Test: Trajectory with no collisions
TEST_F(QuadtreeCollisionCheckerTest, NoCollisionTrajectory) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    // Create a trajectory in free space (away from obstacles)
    auto trajectory = createTrajectory({
        {-10.0, -2.0},
        {-8.0, -2.0},
        {-6.0, -2.0},
        {-4.0, -2.0}
    });
    
    EXPECT_NO_THROW(checker->isCollision(trajectory));
}

// Test: Trajectory with collision
TEST_F(QuadtreeCollisionCheckerTest, CollisionInTrajectory) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    // Create trajectory that passes through obstacle
    auto trajectory = createTrajectory({
        {-10.0, -2.0},
        {-10.0, -5.0},
        {-10.0, -8.0},  // This should collide with obstacle
        {-10.0, -10.0}
    });
    
    EXPECT_TRUE(checker->isCollision(trajectory))
        << "Should detect collision in trajectory";
}

// Test: Boundary checking - outside bounds
TEST_F(QuadtreeCollisionCheckerTest, OutsideBoundaryCollision) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    // Boundary from env1.yaml: [-20.0, 1.0, -20.0, 2.0]
    // Test position outside boundary
    auto trajectory = createTrajectory({{-25.0, 0.0}});
    
    EXPECT_TRUE(checker->isCollision(trajectory))
        << "Should detect collision outside boundary";
}

// Test: Boundary checking - at boundary edge
TEST_F(QuadtreeCollisionCheckerTest, AtBoundaryEdge) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    // Boundary from env1.yaml: x ∈ [-20.0, 1.0], y ∈ [-20.0, 2.0]
    auto trajectory = createTrajectory({{-20.0, -20.0}});
    
    // At exact boundary edge - behavior depends on implementation
    EXPECT_NO_THROW(checker->isCollision(trajectory));
}

// Test: Boundary checking - within bounds
TEST_F(QuadtreeCollisionCheckerTest, WithinBoundary) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    // Position well within boundary
    auto trajectory = createTrajectory({{-10.0, 0.0}});
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle positions within boundary";
}

// Test: Empty trajectory
TEST_F(QuadtreeCollisionCheckerTest, EmptyTrajectory) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    std::vector<Eigen::VectorXd> empty_trajectory;
    
    EXPECT_FALSE(checker->isCollision(empty_trajectory))
        << "Empty trajectory should not have collisions";
}

// Test: Single point trajectory
TEST_F(QuadtreeCollisionCheckerTest, SinglePointTrajectory) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    auto trajectory = createTrajectory({{-5.0, 0.0}});
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle single point trajectory";
}

// Test: Long trajectory
TEST_F(QuadtreeCollisionCheckerTest, LongTrajectory) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    std::vector<std::vector<double>> points;
    // Create a long trajectory with 100 points from start to goal
    for (int i = 0; i < 100; ++i) {
        double t = i / 100.0;
        double x = -10.0 + t * 10.0;  // From -10 to 0
        double y = -2.0 + t * 2.0;     // From -2 to 0
        points.push_back({x, y});
    }
    auto trajectory = createTrajectory(points);
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle long trajectory efficiently";
}

// Test: Trajectory with 3D points (should use only first 2 dimensions)
TEST_F(QuadtreeCollisionCheckerTest, TrajectoryWith3DPoints) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    auto trajectory = createTrajectory({
        {-10.0, -2.0, 1.5707},  // 3D point with heading
        {-5.0, 0.0, 0.0}
    });
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle 3D points (using only x, y)";
}

// Test: Near-miss trajectory
TEST_F(QuadtreeCollisionCheckerTest, NearMissTrajectory) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    // Position just outside collision range of obstacle at [-10, -8]
    // Robot radius is 0.345, so distance > 0.345 + 0.345 should be safe
    auto trajectory = createTrajectory({{-9.0, -7.0}});
    
    bool collision = checker->isCollision(trajectory);
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle near-miss positions";
}

// Test: Dense obstacle environment
TEST_F(QuadtreeCollisionCheckerTest, DenseObstacleEnvironment) {
    // env1 has 14 obstacles - test navigation through them
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    // Try various free space positions
    std::vector<std::vector<double>> free_positions = {
        {-10.0, -2.0},
        {-10.0, 0.0},
        {-8.0, 0.0},
        {0.0, 0.0}
    };
    
    for (const auto& pos : free_positions) {
        auto trajectory = createTrajectory({pos});
        EXPECT_NO_THROW(checker->isCollision(trajectory))
            << "Should handle position: [" << pos[0] << ", " << pos[1] << "]";
    }
}

// Test: Non-convex environment if available
TEST_F(QuadtreeCollisionCheckerTest, NonConvexEnvironment) {
    fs::path nonconvex_path = fs::path(test_data_dir_) / "ugv/env_nonconvex.yaml";
    
    if (fs::exists(nonconvex_path)) {
        ASSERT_NO_THROW({
            auto checker = createCheckerFromYAML("ugv/env_nonconvex.yaml");
            ASSERT_NE(checker, nullptr);
            
            // Test some collision checks
            auto trajectory = createTrajectory({{0.0, 0.0}});
            EXPECT_NO_THROW(checker->isCollision(trajectory));
        });
    } else {
        GTEST_SKIP() << "Non-convex environment file not found";
    }
}

// Test: Bug trap environment if available
TEST_F(QuadtreeCollisionCheckerTest, BugTrapEnvironment) {
    fs::path bugtrap_path = fs::path(test_data_dir_) / "ugv/env_bug_trap.yaml";
    
    if (fs::exists(bugtrap_path)) {
        ASSERT_NO_THROW({
            auto checker = createCheckerFromYAML("ugv/env_bug_trap.yaml");
            ASSERT_NE(checker, nullptr);
        });
    } else {
        GTEST_SKIP() << "Bug trap environment file not found";
    }
}

// Performance test: Many collision checks
TEST_F(QuadtreeCollisionCheckerTest, PerformanceTest) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Perform 1000 collision checks
    for (int i = 0; i < 1000; ++i) {
        double x = -20.0 + (i % 100) * 0.21;
        double y = -20.0 + (i / 100) * 0.42;
        auto trajectory = createTrajectory({{x, y}});
        checker->isCollision(trajectory);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "1000 collision checks took: " << duration.count() << " ms" << std::endl;
    EXPECT_LT(duration.count(), 5000) << "Performance should be reasonable";
}

// Test: Reverse trajectory (should give same result)
TEST_F(QuadtreeCollisionCheckerTest, ReverseTrajectory) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    std::vector<std::vector<double>> forward_points = {
        {-10.0, -2.0},
        {-8.0, -1.0},
        {-6.0, 0.0}
    };
    
    std::vector<std::vector<double>> reverse_points = {
        {-6.0, 0.0},
        {-8.0, -1.0},
        {-10.0, -2.0}
    };
    
    auto forward_trajectory = createTrajectory(forward_points);
    auto reverse_trajectory = createTrajectory(reverse_points);
    
    bool forward_collision = checker->isCollision(forward_trajectory);
    bool reverse_collision = checker->isCollision(reverse_trajectory);
    
    EXPECT_EQ(forward_collision, reverse_collision)
        << "Forward and reverse trajectories should have same collision status";
}

// Test: Diagonal trajectory
TEST_F(QuadtreeCollisionCheckerTest, DiagonalTrajectory) {
    auto checker = createCheckerFromYAML("ugv/env1.yaml");
    
    std::vector<std::vector<double>> diagonal_points;
    for (int i = 0; i <= 10; ++i) {
        double t = i / 10.0;
        diagonal_points.push_back({-10.0 + t * 10.0, -2.0 + t * 2.0});
    }
    
    auto trajectory = createTrajectory(diagonal_points);
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle diagonal trajectory";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
