//
// Created by airlab on 1/22/26.
//

#include "octree/OctreeCollisionChecker.h"
#include "octree/OctreeCpp.h"
#include <iostream>

struct OctreeCollisionChecker::Impl {
    struct vec {
        float x, y, z;
        auto operator<=>(const vec&) const = default;
    };
    using BasicOctree = OctreeCpp<vec, float>;

    std::unique_ptr<BasicOctree> octree;
    float robot_radius;
    float max_obstacle_radius;

    Impl(const ParamPtr& pm) {
        auto boundary = pm->get_param<std::array<float, 6>>("boundary");

        // Create boundary
        auto first_half = vec{boundary[0], boundary[1], boundary[2]};
        auto second_half = vec{boundary[3], boundary[4], boundary[5]};

        octree = std::make_unique<BasicOctree>(BasicOctree::TBoundary{ first_half, second_half });

        robot_radius = pm->get_param<float>("robot_radius");
        float obs_len = pm->get_param<float>("obstacle_length");
        max_obstacle_radius = obs_len / 2.0f;

        auto obstacles = pm->get_ndarray<float>("obstacles");
        for (const auto& obs : obstacles) {
            if (obs.size() != 3) {
                throw std::runtime_error("Each obstacle must have exactly 3 coordinates.");
            }
            octree->Add(BasicOctree::TDataWrapper{{obs[0], obs[1], obs[2]}, obs_len});
        }
    }
};

OctreeCollisionChecker::OctreeCollisionChecker(const ParamPtr &pm)
    : BaseCollisionChecker(pm) {
    pimpl_ = std::make_unique<Impl>(pm);
}

OctreeCollisionChecker::~OctreeCollisionChecker() = default;

bool OctreeCollisionChecker::isCollision(const std::vector<Eigen::VectorXd> &trajectory) const {
    for(const auto& point: trajectory){
        if(point.size() < 3){
            throw std::runtime_error("Each point in trajectory must have at least 3 dimensions.");
        }
        Impl::vec query_point{float(point[0]), float(point[1]), float(point[2])};
        
        // Broad phase: Query the octree with an inflated radius
        float search_radius = pimpl_->robot_radius + pimpl_->max_obstacle_radius;
        auto results = pimpl_->octree->Query(Impl::BasicOctree::Sphere{query_point, search_radius});
        
        for (const auto& res : results) {
            // Narrow phase: Precise check for Sphere vs Axis-Aligned Bounding Box (AABB)
            float half_len = res.Data / 2.0f;
            float min_x = res.Vector.x - half_len;
            float max_x = res.Vector.x + half_len;
            float min_y = res.Vector.y - half_len;
            float max_y = res.Vector.y + half_len;
            float min_z = res.Vector.z - half_len;
            float max_z = res.Vector.z + half_len;

            // Find the closest point on the AABB to the query point
            float closest_x = std::max(min_x, std::min(query_point.x, max_x));
            float closest_y = std::max(min_y, std::min(query_point.y, max_y));
            float closest_z = std::max(min_z, std::min(query_point.z, max_z));

            // Calculate distance squared from query point to closest point on AABB
            float dx = query_point.x - closest_x;
            float dy = query_point.y - closest_y;
            float dz = query_point.z - closest_z;
            float dist_sq = dx*dx + dy*dy + dz*dz;

            if (dist_sq <= pimpl_->robot_radius * pimpl_->robot_radius + 1e-6) {
                return true; // Collision detected
            }
        }
    }
    return false; // No collision
}
