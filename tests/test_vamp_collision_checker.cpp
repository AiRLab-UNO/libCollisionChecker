#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <filesystem>
#include <chrono>
#include <Eigen/Dense>
#include "CollisionChecker.hh"
#include "common/ParamManager.h"

namespace fs = std::filesystem;

class VampCollisionCheckerTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_data_dir_ = TEST_DATA_DIR;
    }

    std::shared_ptr<vamp::VampCollisionChecker> createCheckerFromYAML(const std::string& yaml_file) {
        fs::path yaml_path = fs::path(test_data_dir_) / yaml_file;
        EXPECT_TRUE(fs::exists(yaml_path)) << "Test file not found: " << yaml_path;
        
        auto pm = std::make_shared<param_manager>(yaml_path.string());
        return std::make_shared<vamp::VampCollisionChecker>(pm);
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

// Test: Constructor with UAV configuration
TEST_F(VampCollisionCheckerTest, ConstructorWithUAVConfig) {
    ASSERT_NO_THROW({
        auto checker = createCheckerFromYAML("uav/quad1.yaml");
        ASSERT_NE(checker, nullptr);
    });
}

// Test: Multiple UAV configurations
TEST_F(VampCollisionCheckerTest, LoadMultipleUAVConfigs) {
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

// Test: No collision at start position
TEST_F(VampCollisionCheckerTest, NoCollisionAtStart) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Start position from quad1.yaml: [0.0, 0.0, 0.0]
    auto trajectory = createTrajectory({{0.0, 0.0, 0.0}});
    
    EXPECT_FALSE(checker->isCollision(trajectory))
        << "Start position should be collision-free";
}

// Test: No collision at goal position
TEST_F(VampCollisionCheckerTest, NoCollisionAtGoal) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Goal position from quad1.yaml: [2.3, 3.0, 3.5]
    auto trajectory = createTrajectory({{2.3, 3.0, 3.5}});
    
    EXPECT_FALSE(checker->isCollision(trajectory))
        << "Goal position should be collision-free";
}

// Test: Collision at obstacle
TEST_F(VampCollisionCheckerTest, CollisionAtObstacle) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // First obstacle from quad1.yaml: [1, 1, 1]
    auto trajectory = createTrajectory({{1.0, 1.0, 1.0}});
    
    EXPECT_TRUE(checker->isCollision(trajectory))
        << "Should detect collision at obstacle center";
}

// Test: Empty trajectory
TEST_F(VampCollisionCheckerTest, EmptyTrajectory) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    std::vector<Eigen::VectorXd> empty_trajectory;
    
    EXPECT_FALSE(checker->isCollision(empty_trajectory))
        << "Empty trajectory should not have collisions";
}

// Test: Single point trajectory
TEST_F(VampCollisionCheckerTest, SinglePointTrajectory) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    auto trajectory = createTrajectory({{2.0, 2.0, 2.0}});
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle single point trajectory";
}

// Test: SIMD batch processing with exactly SIMD width points
TEST_F(VampCollisionCheckerTest, SIMDBatchSizeTrajectory) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Create trajectory with exactly 8 points (AVX2 width) or 4 points (NEON width)
    std::vector<std::vector<double>> points;
#if defined(__x86_64__)
    const size_t simd_width = 8;  // AVX2
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
    const size_t simd_width = 4;  // NEON
#else
    const size_t simd_width = 4;  // Default
#endif
    
    for (size_t i = 0; i < simd_width; ++i) {
        double t = i / static_cast<double>(simd_width);
        points.push_back({t * 2.0, t * 2.0, t * 2.0});
    }
    
    auto trajectory = createTrajectory(points);
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle SIMD-width trajectory efficiently";
}

// Test: Trajectory larger than SIMD width
TEST_F(VampCollisionCheckerTest, LargerThanSIMDWidth) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Create 20-point trajectory (tests multiple SIMD batches)
    std::vector<std::vector<double>> points;
    for (int i = 0; i < 20; ++i) {
        double t = i / 20.0;
        points.push_back({t * 2.0, t * 2.0, t * 2.0});
    }
    
    auto trajectory = createTrajectory(points);
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle multi-batch SIMD processing";
}

// Test: Trajectory not divisible by SIMD width (tests remainder handling)
TEST_F(VampCollisionCheckerTest, NonDivisibleBySIMDWidth) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Create 10-point trajectory (not divisible by 8 on AVX2)
    std::vector<std::vector<double>> points;
    for (int i = 0; i < 10; ++i) {
        double t = i / 10.0;
        points.push_back({t * 2.0, t * 2.0, t * 2.0});
    }
    
    auto trajectory = createTrajectory(points);
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle remainder points correctly";
}

// Test: Trajectory with collision in first SIMD batch
TEST_F(VampCollisionCheckerTest, CollisionInFirstBatch) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // First point collides, rest are free
    std::vector<std::vector<double>> points;
    points.push_back({1.0, 1.0, 1.0});  // Collision
    for (int i = 1; i < 10; ++i) {
        points.push_back({0.0, 0.0, static_cast<double>(i) * 0.1});
    }
    
    auto trajectory = createTrajectory(points);
    EXPECT_TRUE(checker->isCollision(trajectory))
        << "Should detect collision in first batch";
}

// Test: Trajectory with collision in last SIMD batch
TEST_F(VampCollisionCheckerTest, CollisionInLastBatch) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Last point collides, rest are free
    std::vector<std::vector<double>> points;
    for (int i = 0; i < 9; ++i) {
        points.push_back({0.0, 0.0, static_cast<double>(i) * 0.1});
    }
    points.push_back({1.0, 1.0, 1.0});  // Collision
    
    auto trajectory = createTrajectory(points);
    EXPECT_TRUE(checker->isCollision(trajectory))
        << "Should detect collision in last batch";
}

// Test: Trajectory with collision in remainder points
TEST_F(VampCollisionCheckerTest, CollisionInRemainder) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Create trajectory where collision is in the remainder
    std::vector<std::vector<double>> points;
    for (int i = 0; i < 8; ++i) {  // First batch is clear
        points.push_back({0.0, 0.0, static_cast<double>(i) * 0.1});
    }
    // Add remainder with collision
    points.push_back({1.0, 1.0, 1.0});  // Collision in remainder
    
    auto trajectory = createTrajectory(points);
    EXPECT_TRUE(checker->isCollision(trajectory))
        << "Should detect collision in remainder points";
}

// Test: Long trajectory for SIMD performance
TEST_F(VampCollisionCheckerTest, LongTrajectory) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    std::vector<std::vector<double>> points;
    // Create a long trajectory with 100 points
    for (int i = 0; i < 100; ++i) {
        double t = i / 100.0;
        points.push_back({t * 2.0, t * 2.0, t * 2.0});
    }
    auto trajectory = createTrajectory(points);
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle long trajectory efficiently with SIMD";
}

// Test: Performance comparison - SIMD should be faster
TEST_F(VampCollisionCheckerTest, PerformanceTest) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Create a large trajectory for performance testing
    std::vector<std::vector<double>> points;
    for (int i = 0; i < 1000; ++i) {
        double t = i / 1000.0;
        points.push_back({t * 4.0, t * 4.0, t * 4.0});
    }
    auto trajectory = createTrajectory(points);
    
    auto start = std::chrono::high_resolution_clock::now();
    bool collision = checker->isCollision(trajectory);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    std::cout << "VAMP SIMD collision check for 1000 points took: " 
              << duration.count() << " microseconds" << std::endl;
    
    EXPECT_LT(duration.count(), 50000)  // Should be very fast with SIMD
        << "SIMD optimization should provide good performance";
}

// Test: Multiple collision checks (batch performance)
TEST_F(VampCollisionCheckerTest, BatchPerformanceTest) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Perform 100 collision checks on 100-point trajectories
    for (int iteration = 0; iteration < 100; ++iteration) {
        std::vector<std::vector<double>> points;
        for (int i = 0; i < 100; ++i) {
            double t = i / 100.0;
            double offset = iteration * 0.01;
            points.push_back({t * 2.0 + offset, t * 2.0 + offset, t * 2.0 + offset});
        }
        auto trajectory = createTrajectory(points);
        checker->isCollision(trajectory);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "100 collision checks on 100-point trajectories took: " 
              << duration.count() << " ms" << std::endl;
    
    EXPECT_LT(duration.count(), 5000)
        << "Batch processing should be efficient";
}

// Test: 2D trajectory (z=0) compatibility
TEST_F(VampCollisionCheckerTest, TwoDimensionalTrajectory) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Create 2D trajectory
    auto trajectory = createTrajectory({
        {0.0, 0.0},
        {1.0, 1.0},
        {2.0, 2.0}
    });
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle 2D trajectories (z defaults to 0)";
}

// Test: Trajectory with multiple obstacles
TEST_F(VampCollisionCheckerTest, MultipleObstacleCollisions) {
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

// Test: Near-miss scenarios
TEST_F(VampCollisionCheckerTest, NearMissScenarios) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Position just outside collision range of obstacle at [1, 1, 1]
    auto trajectory = createTrajectory({{0.3, 0.3, 0.3}});
    
    EXPECT_FALSE(checker->isCollision(trajectory))
        << "Should not detect collision just outside range";
}

// Test: Trajectory passing between obstacles
TEST_F(VampCollisionCheckerTest, PassingBetweenObstacles) {
    auto checker = createCheckerFromYAML("uav/quad1.yaml");
    
    // Create trajectory that passes between obstacles
    auto trajectory = createTrajectory({
        {0.0, 0.0, 0.0},
        {2.0, 2.0, 1.5},  // Between obstacles at z=1 and z=3
        {2.3, 3.0, 3.5}   // Goal
    });
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle trajectory between obstacles";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
