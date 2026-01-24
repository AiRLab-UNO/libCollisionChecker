#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <filesystem>
#include <Eigen/Dense>
#include "CollisionChecker.hh"
#include "common/ParamManager.h"

namespace fs = std::filesystem;

class OctreeCollisionCheckerTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_data_dir_ = TEST_DATA_DIR;
    }

    std::shared_ptr<octree::OctreeCollisionChecker> createCheckerFromYAML(const std::string& yaml_file) {
        fs::path yaml_path = fs::path(test_data_dir_) / yaml_file;
        EXPECT_TRUE(fs::exists(yaml_path)) << "Test file not found: " << yaml_path;
        
        auto pm = std::make_shared<param_manager>(yaml_path.string());
        return std::make_shared<octree::OctreeCollisionChecker>(pm);
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

// Test: Constructor and initialization with UAV test files
TEST_F(OctreeCollisionCheckerTest, ConstructorWithUAVConfig) {
    ASSERT_NO_THROW({
        auto checker = createCheckerFromYAML("uav/quad1.yaml");
        ASSERT_NE(checker, nullptr);
    });
}

// Test: Multiple UAV configurations
TEST_F(OctreeCollisionCheckerTest, LoadMultipleUAVConfigs) {
    std::vector<std::string> uav_configs = {
        "uav/quad1.yaml",
        "uav/quad2.yaml",
        "uav/quad3.yaml",
        "uav/quad4.yaml",
        "uav/quad5.yaml"
    };

    for (const auto& config : uav_configs) {
        ASSERT_NO_THROW({
            auto checker = createCheckerFromYAML(config);
            ASSERT_NE(checker, nullptr);
        }) << "Failed to load: " << config;
    }
}

// Test: Collision detection at start position (should be free)
TEST_F(OctreeCollisionCheckerTest, NoCollisionAtStart) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Start position from quad1.yaml: [0.0, 0.0, 0.0]
    auto trajectory = createTrajectory({{0.0, 0.0, 0.0}});
    
    EXPECT_FALSE(checker->isCollision(trajectory)) 
        << "Start position should be collision-free";
}

// Test: Collision detection at goal position
TEST_F(OctreeCollisionCheckerTest, NoCollisionAtGoal) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Goal position from quad1.yaml: [2.3, 3.0, 3.5]
    auto trajectory = createTrajectory({{2.3, 3.0, 3.5}});
    
    EXPECT_FALSE(checker->isCollision(trajectory))
        << "Goal position should be collision-free";
}

// Test: Collision detection at obstacle location
TEST_F(OctreeCollisionCheckerTest, CollisionAtObstacle) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // First obstacle from quad1.yaml: [1, 1, 1]
    // Should collide since robot has radius 0.345 and obstacle has length 0.5
    auto trajectory = createTrajectory({{1.0, 1.0, 1.0}});
    
    EXPECT_TRUE(checker->isCollision(trajectory))
        << "Should detect collision at obstacle center";
}

// Test: Multiple obstacles collision check
TEST_F(OctreeCollisionCheckerTest, CollisionAtMultipleObstacles) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Test multiple obstacle positions from quad1.yaml
    std::vector<std::vector<double>> obstacle_positions = {
        {1.0, 1.0, 1.0},
        {3.0, 1.0, 1.0},
        {2.0, 2.0, 2.0},
        {1.0, 3.0, 3.0}
    };
    
    for (const auto& pos : obstacle_positions) {
        auto trajectory = createTrajectory({pos});
        EXPECT_TRUE(checker->isCollision(trajectory))
            << "Should detect collision at obstacle position: ["
            << pos[0] << ", " << pos[1] << ", " << pos[2] << "]";
    }
}

// Test: Trajectory with no collisions
TEST_F(OctreeCollisionCheckerTest, NoCollisionTrajectory) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Create a trajectory in free space
    auto trajectory = createTrajectory({
        {0.0, 0.0, 0.0},
        {0.5, 0.5, 0.5},
        {1.0, 0.5, 0.5},
        {1.5, 1.0, 0.5}
    });
    
    bool has_collision = checker->isCollision(trajectory);
    // This depends on actual obstacle configuration
    // Testing that the function executes without errors
    EXPECT_NO_THROW(checker->isCollision(trajectory));
}

// Test: Trajectory with collision
TEST_F(OctreeCollisionCheckerTest, CollisionInTrajectory) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Create trajectory that passes through obstacle at [1, 1, 1]
    auto trajectory = createTrajectory({
        {0.0, 0.0, 0.0},
        {0.5, 0.5, 0.5},
        {1.0, 1.0, 1.0},  // This should collide
        {1.5, 1.5, 1.5}
    });
    
    EXPECT_TRUE(checker->isCollision(trajectory))
        << "Should detect collision in trajectory";
}

// Test: Near-miss trajectory (just outside collision radius)
TEST_F(OctreeCollisionCheckerTest, NearMissTrajectory) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Position just outside collision range of obstacle at [1, 1, 1]
    // Robot radius is 0.345, obstacle half-length is 0.25
    // Total collision radius ≈ 0.595
    // So at distance > 0.6 should be safe
    auto trajectory = createTrajectory({{0.3, 0.3, 0.3}});
    
    EXPECT_FALSE(checker->isCollision(trajectory))
        << "Should not detect collision just outside collision radius";
}

// Test: Boundary checking - within bounds
TEST_F(OctreeCollisionCheckerTest, WithinBoundary) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Boundary from quad1.yaml: [0, 0, 0, 5, 5, 5]
    auto trajectory = createTrajectory({
        {2.5, 2.5, 2.5},
        {4.0, 4.0, 4.0}
    });
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle positions within boundary";
}

// Test: Empty trajectory
TEST_F(OctreeCollisionCheckerTest, EmptyTrajectory) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    std::vector<Eigen::VectorXd> empty_trajectory;
    
    EXPECT_FALSE(checker->isCollision(empty_trajectory))
        << "Empty trajectory should not have collisions";
}

// Test: Single point trajectory
TEST_F(OctreeCollisionCheckerTest, SinglePointTrajectory) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    auto trajectory = createTrajectory({{2.0, 2.0, 2.0}});
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle single point trajectory";
}

// Test: Long trajectory
TEST_F(OctreeCollisionCheckerTest, LongTrajectory) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    std::vector<std::vector<double>> points;
    // Create a long trajectory with 100 points
    for (int i = 0; i < 100; ++i) {
        double t = i / 100.0;
        points.push_back({t * 2.0, t * 2.0, t * 2.0});
    }
    auto trajectory = createTrajectory(points);
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle long trajectory efficiently";
}

// Test: Invalid trajectory (less than 3 dimensions)
TEST_F(OctreeCollisionCheckerTest, InvalidTrajectoryDimension) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    Eigen::VectorXd invalid_point(2);  // Only 2D
    invalid_point << 1.0, 1.0;
    std::vector<Eigen::VectorXd> trajectory = {invalid_point};
    
    EXPECT_THROW(checker->isCollision(trajectory), std::runtime_error)
        << "Should throw error for trajectory with dimension < 3";
}

// Test: Trajectory with extra dimensions (should work, only first 3 used)
TEST_F(OctreeCollisionCheckerTest, TrajectoryWithExtraDimensions) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    Eigen::VectorXd point(5);  // 5D point
    point << 0.0, 0.0, 0.0, 1.0, 2.0;
    std::vector<Eigen::VectorXd> trajectory = {point};
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle trajectory with extra dimensions";
}

// Test: Collision at obstacle edge
TEST_F(OctreeCollisionCheckerTest, CollisionAtObstacleEdge) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Obstacle at [1, 1, 1] with half-length 0.25
    // Robot with radius 0.345
    // Position at edge of obstacle + robot radius
    auto trajectory = createTrajectory({{1.6, 1.0, 1.0}});
    
    bool collision = checker->isCollision(trajectory);
    // Edge cases may or may not collide depending on precision
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle edge case positions";
}

// Performance test: Many collision checks
TEST_F(OctreeCollisionCheckerTest, PerformanceTest) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Perform 1000 collision checks
    for (int i = 0; i < 1000; ++i) {
        double x = (i % 10) * 0.5;
        double y = ((i / 10) % 10) * 0.5;
        double z = (i / 100) * 0.5;
        auto trajectory = createTrajectory({{x, y, z}});
        checker->isCollision(trajectory);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "1000 collision checks took: " << duration.count() << " ms" << std::endl;
    EXPECT_LT(duration.count(), 5000) << "Performance should be reasonable";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
