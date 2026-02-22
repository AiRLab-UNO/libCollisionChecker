#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <filesystem>
#include "../include/common/ParamManager.h"
#include "../include/common/FindCollisionChecker.h"
const std::string env1 = "/home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/test/ugv/env1.yaml";
const std::string quad1 = "/home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/test/uav/quad1.yaml";
const std::string img1 = "/home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/test/ugv/sr_clutter_01.yaml";
const std::string tri1 = "/home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/test/ugv/env_bug_trap.yaml";
const std::string rect1 = "/home/redwan/research/2026/ResearchWorkspace/BOT-Connect/Code/libcollisionchecker/test/ugv/env_rectangles.yaml";

// Defining a Test Case
TEST(FindCollisionChecker, PlanGeneration) {

    auto params = std::make_shared<param_manager>(env1);
    FindCollisionChecker finder(params);
    auto plan = finder.get_plan();
    for(int step : plan) {
        std::cout << step << " ";
    }
}

TEST(FindCollisionChecker, AvailableCheckers) {
    for(auto env : {env1, quad1, img1, tri1, rect1}) {
         auto params = std::make_shared<param_manager>(env);
        FindCollisionChecker finder(params);
        auto plan = finder.get_plan();
        auto checkers = finder.available_checkers(plan);
        std::cout << "Environment: " << env << std::endl;
        finder.print_available_checkers(checkers);
    }
}



int main(int argc, char **argv) {
    // Initializes the Google Test framework
    ::testing::InitGoogleTest(&argc, argv);

    // Runs all tests defined in this project
    return RUN_ALL_TESTS();
}
