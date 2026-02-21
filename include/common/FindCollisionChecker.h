//
// Created by redwan on 2/21/26.
//

#ifndef LIBCOLLISIONCHECKER_FINDCOLLISIONCHECKER_H
#define LIBCOLLISIONCHECKER_FINDCOLLISIONCHECKER_H
#include <iostream>
#include <map>
#include <string>
#include "ParamManager.h"
/*
 * Factory function to create collision checker instances based on the specification
 digraph G {
obstacle -> box
obstacle -> triangles
obstacle -> image

box -> dim_2D
box -> dim_3D


dim_2D -> rectangle
dim_2D -> square

dim_3D -> cube
dim_3D -> cuboid

image -> OCCUPANCY

rectangle -> FCL
rectangle -> VAMP
rectangle -> QUADTREE

square -> FCL
square -> VAMP
square -> QUADTREE


cube -> OCTREE
cube -> FCL
cube -> VAMP


cuboid -> OCTREE
cuboid -> FCL
cuboid -> VAMP

triangles ->FCL

}
 */
enum class collision_checker_type {
    FCL = 10,
    VAMP,
    QUADTREE,
    OCTREE,
    OCCUPANCY
};

enum obstacle_type {
    BOX = 1,
    TRIANGLES,
    IMAGE
};

enum dimension_type {
    DIM_2D = 4,
    DIM_3D
};

enum shape_type {
    RECTANGLE = 6,
    SQUARE,
    CUBE,
    CUBOID
};

class FindCollisionChecker {
public:
    FindCollisionChecker(const ParamPtr& pm): pm_(pm) {
        populate_state_transition_graph();
    }
    std::vector<int> get_plan()
    {
        std::vector<int> plan;
        plan.push_back(getObstacleType());
        plan.push_back(getDimensionType());
        plan.push_back(getShapeType());
        return plan;
    }

    std::vector<collision_checker_type> available_checkers(const std::vector<int>& plan) {
        std::vector<collision_checker_type> checkers;
        for (int node : plan) {
            if (graph_.find(node) != graph_.end()) {
                for (int checker : graph_[node]) {
                    if(checker >= 10) // Ensure it's a valid checker type
                        checkers.push_back(static_cast<collision_checker_type>(checker));
                }
            }
        }
        return checkers;
    }

    void print_available_checkers(const std::vector<collision_checker_type>& checkers) {
        std::cout << "Available Collision Checkers: ";
        for (const auto& checker : checkers) {
            switch (checker) {
                case collision_checker_type::OCCUPANCY:
                    std::cout << "OCCUPANCY ";
                    break;
                case collision_checker_type::FCL:
                    std::cout << "FCL ";
                    break;
                case collision_checker_type::VAMP:
                    std::cout << "VAMP ";
                    break;
                case collision_checker_type::QUADTREE:
                    std::cout << "QUADTREE ";
                    break;
                case collision_checker_type::OCTREE:
                    std::cout << "OCTREE ";
                    break;
                default:
                    std::cout << "UNKNOWN ";
            }
        }
        std::cout << std::endl;
    }


private:
    ParamPtr pm_;
    std::map<int, std::vector<int>> graph_;
protected:
    void populate_state_transition_graph() {
        // Populate the graph based on the provided structure
        graph_[0] = {1, 2, 3}; // obstacle -> box, triangles, image
        graph_[1] = {4, 5};    // box -> dim_2D, dim_3D
        graph_[4] = {6, 7};    // dim_2D -> rectangle, square
        graph_[5] = {8, 9};    // dim_3D -> cube, cuboid
        graph_[6] = {10, 11, 12}; // rectangle -> FCL, VAMP, QUADTREE
        graph_[7] = {10, 11, 12}; // square -> FCL, VAMP, QUADTREE
        graph_[8] = {13, 10, 11}; // cube -> OCTREE, FCL, VAMP
        graph_[9] = {10, 11}; // cuboid ->  FCL, VAMP
        graph_[2] = {10};       // triangles -> FCL
        graph_[3] = {14};       // image -> OCCUPANCY
    }
public:
    int getDimensionType() {
        int obstacleType = getObstacleType();
        if(obstacleType != BOX)
            return -1; // Dimension type is only relevant for box obstacles
        if(pm_->has_param("obstacles")){
            auto obsList = pm_->get_ndarray<float>("obstacles");
            if (!obsList.empty()) {
                if( obsList[0].size() == 2 || obsList[0].size() == 4) {
                    // If the first obstacle has 2 or 4 parameters, we can infer it's 2D (x, y) or (x, y, width, height)
                    return DIM_2D;
                }
                return DIM_3D;
            }

        }

        return -1; // Invalid type
    }
    int getObstacleType() {
        if(pm_->has_param("triangles")){
            return TRIANGLES;
        }
        else if (pm_->has_param("map_image_path")){
            return IMAGE;
        }
        else if(pm_->has_param("obstacles")){
            return BOX;
        }
        return -1; // Invalid type
    }
    int getShapeType() {
        int dim = getDimensionType();
        if(dim == -1) {
            return -1; // Invalid type
        }
        auto obsList = pm_->get_ndarray<float>("obstacles");

        if (dim == DIM_2D) {
            // For 2D, we can further check if it's a rectangle or square based on the parameters
            if (!obsList.empty() && obsList[0].size() == 4) {
                return RECTANGLE; // (x, y, width, height)
            }
            else if(pm_->has_param("obstacle_length"))
                return SQUARE; // (x, y) with a fixed size
        } else if (dim == DIM_3D) {
            // For 3D, we can check if it's a cube or cuboid based on the parameters
            if (!obsList.empty() && obsList[0].size() == 6) {
                return CUBOID; // (x, y, z, width, height, depth)
            }
            else if(pm_->has_param("obstacle_length"))
                return CUBE; // (x, y, z) with a fixed size
        }
        return -1; // Invalid type
    }
};

#endif //LIBCOLLISIONCHECKER_FINDCOLLISIONCHECKER_H
