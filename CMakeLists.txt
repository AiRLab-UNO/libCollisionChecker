cmake_minimum_required(VERSION 3.22)
project(libCollisionChecker)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)
endif()

include(cmake/CPM.cmake)
CPMAddPackage(
    NAME YAML-CPP
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
)
include_directories(include)
find_package(Eigen3 REQUIRED)

option(BUILD_CC_OCTREE "BUILD OCTREE COLLISION CHECKER" ON)
option(BUILD_CC_QUAD_TREE "BUILD QUADTREE COLLISION CHECKER" ON)
option(BUILD_CC_OCCUPANCY_MAP "BUILD OCCUPANCY MAP COLLISION CHECKER" ON)
option(BUILD_CC_FCL "BUILD FCL COLLISION CHECKER" ON)
option(BUILD_CC_VAMP "BUILD VAMP COLLISION CHECKER" ON)
option(BUILD_TESTING "BUILD TESTS" ON)


set(CC_VAMP
        src/VAMP/VampCollisionChecker.cpp
)

set(CC_QUAD_TREE
        src/quadtree/QuadtreeCollisionChecker.cpp
)

set(CC_OCTREE
        src/octree/OctreeCollisionChecker.cpp
)

set(CC_FCL
        src/FCL/FCL.cpp
        include/FCL/FCL.h
)

set(CC_OCCUPANCY_MAP
        src/occupancy/OccupancyMap.cpp
        src/occupancy/lodepng.cpp
)

if(BUILD_CC_OCTREE)
    add_library(OctreeCollisionChecker SHARED ${CC_OCTREE})
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

if(BUILD_CC_VAMP)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
    # vamp integration
    include(CompilerSettings)
    include(FetchInitCPM)
    include(Dependencies)
    list(APPEND VAMP_EXT_INCLUDES
            ${NIGH_INCLUDE_DIRS}
            ${PDQSORT_INCLUDE_DIRS}
    )
    include_directories(${VAMP_EXT_INCLUDES})
    # Add VAMP_EXT_SOURCES if available (for SIMD support on x86_64)
    if(VAMP_EXT_SOURCES)
        list(APPEND CC_VAMP ${VAMP_EXT_SOURCES})
    endif()
    add_library(VampCollisionChecker SHARED ${CC_VAMP})
    target_link_libraries(VampCollisionChecker Eigen3::Eigen yaml-cpp)
    target_include_directories(VampCollisionChecker PUBLIC include/vamp PRIVATE src/VAMP ${VAMP_EXT_INCLUDES})
endif()

# Enable testing if requested
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()