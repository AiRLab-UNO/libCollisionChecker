cmake_minimum_required(VERSION 3.30)
project(libCollisionChecker)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)
endif()


include_directories(include)
find_package(Eigen3 REQUIRED)

option(BUILD_CC_OCTREE "BUILD OCTREE COLLISION CHECKER" OFF)
option(BUILD_CC_QUAD_TREE "BUILD QUADTREE COLLISION CHECKER" OFF)
option(BUILD_CC_OCCUPANCY_MAP "BUILD OCCUPANCY MAP COLLISION CHECKER" OFF)
option(BUILD_CC_FCL "BUILD FCL COLLISION CHECKER" ON)


set(CC_VAMP
        src/VAMP/VampCollisionChecker.cpp
        include/vamp/VampCollisionChecker.h
)

set(CC_QUAD_TREE
        src/quadtree/QuadtreeCollisionChecker.cpp
        include/quadtree/QuadtreeCollisionChecker.h
)

set(CC_OCTREE
        src/octree/OctreeCollisionChecker.cpp
        include/octree/OctreeCollisionChecker.h
)

set(CC_FCL
        src/FCL/FCL.cpp
        include/FCL/FCL.h
)

set(CC_OCCUPANCY_MAP
        src/occupancy/OccupancyMap.cpp
        include/occupancy/OccupancyMap.h
)

if(BUILD_CC_OCTREE)
    add_library(OctreeCollisionChecker SHARED ${CC_QUAD_TREE})
    target_link_libraries(OctreeCollisionChecker Eigen3::Eigen yaml-cpp)
    target_include_directories(OctreeCollisionChecker PUBLIC include/octree PRIVATE src/octree)
endif()


if(BUILD_CC_QUAD_TREE)
    add_library(QuadtreeCollisionChecker SHARED ${CC_QUAD_TREE})
    target_link_libraries(QuadtreeCollisionChecker Eigen3::Eigen yaml-cpp)
    target_include_directories(QuadtreeCollisionChecker PUBLIC include/quadtree PRIVATE src/quadtree)
endif()


if(BUILD_CC_OCCUPANCY_MAP)
    add_library(OccupancyMap SHARED ${CC_OCCUPANCY_MAP})
    target_link_libraries(OccupancyMap Eigen3::Eigen yaml-cpp)
    target_include_directories(OccupancyMap PUBLIC include/occupancy PRIVATE src/occupancy)
endif()


if(BUILD_CC_FCL)
    find_package(YAML-CPP REQUIRED)
    find_package(FCL REQUIRED)
    find_package(CCD REQUIRED)
    add_library(FCL SHARED ${CC_FCL})
    target_link_libraries(FCL
            Eigen3::Eigen
            ${FCL_LIBRARIES}
            ${CCD_LIBRARIES}
            yaml-cpp)
    target_include_directories(FCL PUBLIC include/FCL PRIVATE src/FCL)
endif()