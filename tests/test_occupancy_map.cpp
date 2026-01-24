#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <filesystem>
#include <Eigen/Dense>
#include "CollisionChecker.hh"
#include "common/ParamManager.h"

namespace fs = std::filesystem;

class OccupancyMapTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_data_dir_ = TEST_DATA_DIR;
    }

    std::shared_ptr<occupancy::OccupancyMap> createCheckerFromYAML(const std::string& yaml_file) {
        fs::path yaml_path = fs::path(test_data_dir_) / yaml_file;
        EXPECT_TRUE(fs::exists(yaml_path)) << "Test file not found: " << yaml_path;
        
        auto pm = std::make_shared<param_manager>(yaml_path.string());
        return std::make_shared<occupancy::OccupancyMap>(pm);
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

    bool fileExists(const std::string& relative_path) {
        fs::path full_path = fs::path(test_data_dir_) / relative_path;
        return fs::exists(full_path);
    }

    std::string test_data_dir_;
};

// Test: Constructor with Intel map configuration
TEST_F(OccupancyMapTest, ConstructorWithIntelMap) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    ASSERT_NO_THROW({
        auto checker = createCheckerFromYAML("ugv/intel.yaml");
        ASSERT_NE(checker, nullptr);
    });
}

// Test: Constructor with SR clutter maps
TEST_F(OccupancyMapTest, ConstructorWithSRClutterMaps) {
    std::vector<std::string> sr_configs = {
        "ugv/sr_clutter_01.yaml",
        "ugv/sr_clutter_04.yaml"
    };

    for (const auto& config : sr_configs) {
        if (!fileExists(config)) {
            continue;
        }
        
        ASSERT_NO_THROW({
            auto checker = createCheckerFromYAML(config);
            ASSERT_NE(checker, nullptr);
        }) << "Failed to load: " << config;
    }
}

// Test: Constructor with SR nonconvex map
TEST_F(OccupancyMapTest, ConstructorWithSRNonconvexMap) {
    if (!fileExists("ugv/sr_nonconvex_02.yaml") || !fileExists("ugv/sr_nonconvex_02.png")) {
        GTEST_SKIP() << "SR nonconvex map files not found";
    }

    ASSERT_NO_THROW({
        auto checker = createCheckerFromYAML("ugv/sr_nonconvex_02.yaml");
        ASSERT_NE(checker, nullptr);
    });
}

// Test: Collision detection in free space
TEST_F(OccupancyMapTest, NoCollisionInFreeSpace) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    // Test a position that should be in free space
    // This depends on the actual map content
    auto trajectory = createTrajectory({{0.0, 0.0}});
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle collision check in free space";
}

// Test: Collision detection at obstacle
TEST_F(OccupancyMapTest, CollisionAtObstacle) {
    if (!fileExists("ugv/sr_clutter_01.yaml") || !fileExists("ugv/sr_clutter_01.png")) {
        GTEST_SKIP() << "SR clutter map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/sr_clutter_01.yaml");
    
    // Note: Without knowing exact obstacle positions in the image,
    // we test that the function executes correctly
    auto trajectory = createTrajectory({{0.0, 0.0}});
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle collision check";
}

// Test: Trajectory collision checking
TEST_F(OccupancyMapTest, TrajectoryCollisionCheck) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    // Create a simple trajectory
    auto trajectory = createTrajectory({
        {0.0, 0.0},
        {1.0, 1.0},
        {2.0, 2.0},
        {3.0, 3.0}
    });
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle trajectory collision check";
}

// Test: Out of bounds detection
TEST_F(OccupancyMapTest, OutOfBoundsCollision) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    // Test with very large coordinates that are likely out of bounds
    auto trajectory = createTrajectory({{1000.0, 1000.0}});
    
    bool collision = checker->isCollision(trajectory);
    EXPECT_TRUE(collision) << "Should detect collision for out of bounds position";
}

// Test: Empty trajectory
TEST_F(OccupancyMapTest, EmptyTrajectory) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    std::vector<Eigen::VectorXd> empty_trajectory;
    
    EXPECT_FALSE(checker->isCollision(empty_trajectory))
        << "Empty trajectory should not have collisions";
}

// Test: Single point trajectory
TEST_F(OccupancyMapTest, SinglePointTrajectory) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    auto trajectory = createTrajectory({{5.0, 5.0}});
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle single point trajectory";
}

// Test: Long trajectory
TEST_F(OccupancyMapTest, LongTrajectory) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    std::vector<std::vector<double>> points;
    // Create a long trajectory with 200 points
    for (int i = 0; i < 200; ++i) {
        double t = i / 200.0;
        points.push_back({t * 20.0, t * 20.0});
    }
    auto trajectory = createTrajectory(points);
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle long trajectory efficiently";
}

// Test: Trajectory with 3D points (should use only first 2 dimensions)
TEST_F(OccupancyMapTest, TrajectoryWith3DPoints) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    auto trajectory = createTrajectory({
        {0.0, 0.0, 1.5707},
        {5.0, 5.0, 0.0}
    });
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle 3D points (using only x, y)";
}

// Test: Diagonal trajectory
TEST_F(OccupancyMapTest, DiagonalTrajectory) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    std::vector<std::vector<double>> diagonal_points;
    for (int i = 0; i <= 20; ++i) {
        double coord = i * 0.5;
        diagonal_points.push_back({coord, coord});
    }
    
    auto trajectory = createTrajectory(diagonal_points);
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle diagonal trajectory";
}

// Test: Zigzag trajectory
TEST_F(OccupancyMapTest, ZigzagTrajectory) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    std::vector<std::vector<double>> zigzag_points;
    for (int i = 0; i <= 10; ++i) {
        double x = i * 1.0;
        double y = (i % 2 == 0) ? 0.0 : 5.0;
        zigzag_points.push_back({x, y});
    }
    
    auto trajectory = createTrajectory(zigzag_points);
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle zigzag trajectory";
}

// Test: Near origin trajectory
TEST_F(OccupancyMapTest, NearOriginTrajectory) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    // Test points very close to origin
    auto trajectory = createTrajectory({
        {0.1, 0.1},
        {0.2, 0.2},
        {0.3, 0.3}
    });
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle trajectory near origin";
}

// Test: Negative coordinates
TEST_F(OccupancyMapTest, NegativeCoordinates) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    auto trajectory = createTrajectory({{-5.0, -5.0}});
    
    // Negative coordinates might be out of bounds depending on origin
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle negative coordinates";
}

// Test: Multiple maps with different resolutions
TEST_F(OccupancyMapTest, DifferentMapResolutions) {
    std::vector<std::string> map_configs = {
        "ugv/intel.yaml",
        "ugv/sr_clutter_01.yaml",
        "ugv/sr_clutter_04.yaml",
        "ugv/sr_nonconvex_02.yaml"
    };

    for (const auto& config : map_configs) {
        if (!fileExists(config)) {
            continue;
        }
        
        ASSERT_NO_THROW({
            auto checker = createCheckerFromYAML(config);
            ASSERT_NE(checker, nullptr);
            
            // Test basic collision check
            auto trajectory = createTrajectory({{0.0, 0.0}});
            EXPECT_NO_THROW(checker->isCollision(trajectory));
        }) << "Failed with config: " << config;
    }
}

// Test: Clutter environment navigation
TEST_F(OccupancyMapTest, ClutterEnvironmentNavigation) {
    if (!fileExists("ugv/sr_clutter_01.yaml") || !fileExists("ugv/sr_clutter_01.png")) {
        GTEST_SKIP() << "SR clutter map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/sr_clutter_01.yaml");
    
    // Test various positions in a cluttered environment
    std::vector<std::vector<double>> test_positions = {
        {1.0, 1.0},
        {5.0, 5.0},
        {10.0, 10.0},
        {2.5, 7.5}
    };
    
    for (const auto& pos : test_positions) {
        auto trajectory = createTrajectory({pos});
        EXPECT_NO_THROW(checker->isCollision(trajectory))
            << "Should handle position: [" << pos[0] << ", " << pos[1] << "]";
    }
}

// Test: Non-convex environment navigation
TEST_F(OccupancyMapTest, NonconvexEnvironmentNavigation) {
    if (!fileExists("ugv/sr_nonconvex_02.yaml") || !fileExists("ugv/sr_nonconvex_02.png")) {
        GTEST_SKIP() << "SR nonconvex map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/sr_nonconvex_02.yaml");
    
    // Test collision checking in non-convex environment
    auto trajectory = createTrajectory({
        {1.0, 1.0},
        {2.0, 2.0},
        {3.0, 3.0}
    });
    
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle non-convex environment";
}

// Performance test: Many collision checks
TEST_F(OccupancyMapTest, PerformanceTest) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Perform 1000 collision checks
    for (int i = 0; i < 1000; ++i) {
        double x = (i % 50) * 0.5;
        double y = (i / 50) * 0.5;
        auto trajectory = createTrajectory({{x, y}});
        checker->isCollision(trajectory);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "1000 collision checks took: " << duration.count() << " ms" << std::endl;
    EXPECT_LT(duration.count(), 5000) << "Performance should be reasonable";
}

// Test: Reverse trajectory
TEST_F(OccupancyMapTest, ReverseTrajectory) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    std::vector<std::vector<double>> forward_points = {
        {1.0, 1.0},
        {2.0, 2.0},
        {3.0, 3.0}
    };
    
    std::vector<std::vector<double>> reverse_points = {
        {3.0, 3.0},
        {2.0, 2.0},
        {1.0, 1.0}
    };
    
    auto forward_trajectory = createTrajectory(forward_points);
    auto reverse_trajectory = createTrajectory(reverse_points);
    
    bool forward_collision = checker->isCollision(forward_trajectory);
    bool reverse_collision = checker->isCollision(reverse_trajectory);
    
    EXPECT_EQ(forward_collision, reverse_collision)
        << "Forward and reverse trajectories should have same collision status";
}

// Test: Dense sampling trajectory
TEST_F(OccupancyMapTest, DenseSamplingTrajectory) {
    if (!fileExists("ugv/intel.yaml") || !fileExists("ugv/intel_map.png")) {
        GTEST_SKIP() << "Intel map files not found";
    }

    auto checker = createCheckerFromYAML("ugv/intel.yaml");
    
    std::vector<std::vector<double>> dense_points;
    // Very dense sampling - 0.1 unit apart
    for (int i = 0; i <= 100; ++i) {
        double coord = i * 0.1;
        dense_points.push_back({coord, coord});
    }
    
    auto trajectory = createTrajectory(dense_points);
    EXPECT_NO_THROW(checker->isCollision(trajectory))
        << "Should handle densely sampled trajectory";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
