# Google Test configuration for libcollisionchecker
cmake_minimum_required(VERSION 3.22)

# Find GTest
find_package(GTest REQUIRED)
include(GoogleTest)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/common
    ${GTEST_INCLUDE_DIRS}
)

# Common test dependencies
set(TEST_LIBRARIES
    GTest::GTest
    GTest::Main
    Eigen3::Eigen
    yaml-cpp
)

# Test data directory
set(TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test)
add_compile_definitions(TEST_DATA_DIR="${TEST_DATA_DIR}")

# Octree Collision Checker Tests
if(BUILD_CC_OCTREE)
    add_executable(octree_collision_checker_test
        test_octree_collision_checker.cpp
    )
    target_link_libraries(octree_collision_checker_test
        ${TEST_LIBRARIES}
        OctreeCollisionChecker
    )
    target_include_directories(octree_collision_checker_test PRIVATE
        ${CMAKE_SOURCE_DIR}/include/octree
    )
    gtest_discover_tests(octree_collision_checker_test)
endif()

# Quadtree Collision Checker Tests
if(BUILD_CC_QUAD_TREE)
    add_executable(quadtree_collision_checker_test
        test_quadtree_collision_checker.cpp
    )
    target_link_libraries(quadtree_collision_checker_test
        ${TEST_LIBRARIES}
        QuadtreeCollisionChecker
    )
    target_include_directories(quadtree_collision_checker_test PRIVATE
        ${CMAKE_SOURCE_DIR}/include/quadtree
    )
    gtest_discover_tests(quadtree_collision_checker_test)
endif()

# Occupancy Map Collision Checker Tests
if(BUILD_CC_OCCUPANCY_MAP)
    add_executable(occupancy_map_test
        test_occupancy_map.cpp
    )
    target_link_libraries(occupancy_map_test
        ${TEST_LIBRARIES}
        OccupancyMap
    )
    target_include_directories(occupancy_map_test PRIVATE
        ${CMAKE_SOURCE_DIR}/include/occupancy
    )
    gtest_discover_tests(occupancy_map_test)
endif()

# FCL Collision Checker Tests (if enabled)
if(BUILD_CC_FCL)
    add_executable(fcl_collision_checker_test
        test_fcl_collision_checker.cpp
    )
    target_link_libraries(fcl_collision_checker_test
        ${TEST_LIBRARIES}
        FCL
        ${FCL_LIBRARIES}
        ${CCD_LIBRARIES}
    )
    target_include_directories(fcl_collision_checker_test PRIVATE
        ${CMAKE_SOURCE_DIR}/include/FCL
    )
    gtest_discover_tests(fcl_collision_checker_test)
endif()

# VAMP Collision Checker Tests (if enabled)
if(BUILD_CC_VAMP)
    add_executable(vamp_collision_checker_test
        test_vamp_collision_checker.cpp
    )
    target_link_libraries(vamp_collision_checker_test
        ${TEST_LIBRARIES}
        VampCollisionChecker
    )
    target_include_directories(vamp_collision_checker_test PRIVATE
        ${CMAKE_SOURCE_DIR}/include/vamp
        ${VAMP_EXT_INCLUDES}
    )
    gtest_discover_tests(vamp_collision_checker_test)
endif()

# All tests target
add_custom_target(run_all_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS
        $<$<BOOL:${BUILD_CC_OCTREE}>:octree_collision_checker_test>
        $<$<BOOL:${BUILD_CC_QUAD_TREE}>:quadtree_collision_checker_test>
        $<$<BOOL:${BUILD_CC_OCCUPANCY_MAP}>:occupancy_map_test>
        $<$<BOOL:${BUILD_CC_FCL}>:fcl_collision_checker_test>
        $<$<BOOL:${BUILD_CC_VAMP}>:vamp_collision_checker_test>
)


add_executable(test_find_collision_checker
        test_find_collision_checker.cpp
)
target_link_libraries(test_find_collision_checker
        ${TEST_LIBRARIES}
)
