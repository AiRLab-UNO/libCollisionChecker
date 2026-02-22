//
// Created by airlab on 1/23/26.
// VAMP collision checker with manual collision checking to avoid template issues
//

#include "CollisionChecker.hh"
#include "vamp/collision/environment.hh"
#include "vamp/collision/shapes.hh"
#include "vamp/collision/sphere_cuboid.hh"
#include "vamp/collision/math.hh"
#include <cmath>
#include "common/FindCollisionChecker.h"
namespace vc = vamp::collision;

namespace vamp {

struct VampCollisionChecker::Impl {
    vc::Environment<float> env_;
    float _robotRadius;
    bool is3DObstacles{true};
    std::vector<float> _boundary;

    Impl(const ParamPtr& pm) {
        _robotRadius = pm->get_param<float>("robot_radius");
        _boundary = pm->get_param<std::vector<float>>("boundary");
        initialize_manager(pm);


    }


    void initialize_manager(const ParamPtr& pm) {
             // Use FindCollisionChecker to determine the type of obstacles and load them accordingly
        FindCollisionChecker finder(pm);
        auto plan = finder.get_plan();
        auto checkers = finder.available_checkers(plan);
        // `FCL` is also the name of this class (fcl::FCL). Use the global-scope enum value to avoid
        // colliding with the class name inside namespace fcl.
        if(std::count(checkers.begin(), checkers.end(), collision_checker_type::FCL) == 0) {
            std::cerr << "VAMP is not an available checker for the given environment." << std::endl;
            exit(1);
        }


        if(plan[0] != BOX) {
            std::cerr << "VAMP does not support obstacle type. (make sure obs_type is BOX)" << std::endl;
            exit(1);
        }

        auto obsLen = pm->get_param<float>("obstacle_length");
        // obsLen *= 2.0f; // Convert half-length to full length for FCL box geometry
        for(auto& box : pm->get_ndarray<float>("obstacles")) {
            switch (plan[2]) {
                case RECTANGLE:
                    setBox({box[0], box[1], 0.0f, box[2] / 2.0f, box[3] / 2.0f, obsLen});
                    break;
                case SQUARE:
                    setBox({box[0], box[1], 0.0f, obsLen, obsLen, obsLen});
                    break;
                case CUBE:
                    is3DObstacles = true;
                    setBox({box[0], box[1], box[2], obsLen, obsLen, obsLen});
                    break;
                case CUBOID:
                    is3DObstacles = true;
                    setBox({box[0], box[1], box[2], box[3] / 2.0f, box[4] / 2.0f, box[5] / 2.0f});
                    break;
                default:
                    std::cerr << "Unknown shape type in plan: " << plan[2] << std::endl;
            }
        }
        env_.sort(); // Sort for early termination optimization


    }

    void setBox(const std::array<float, 6>& box){

        // Create cuboid obstacles
        env_.cuboids.push_back(vc::Cuboid<float>(
                box[0], box[1], box[2],
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f,
                box[3], box[4], box[5]
        ));

    }

    // Manual sphere-cuboid collision check for scalar float types
    bool checkSphereCuboidCollision(const vc::Cuboid<float>& cuboid, 
                                     float sx, float sy, float sz, float radius) const {
        const float rsq = radius * radius;
        
        // Vector from sphere center to cuboid center
        const float xs = sx - cuboid.x;
        const float ys = sy - cuboid.y;
        const float zs = sz - cuboid.z;

        // Project onto cuboid axes and clamp to half-extents
        const float a1_proj = std::abs(cuboid.axis_1_x * xs + cuboid.axis_1_y * ys + cuboid.axis_1_z * zs);
        const float a2_proj = std::abs(cuboid.axis_2_x * xs + cuboid.axis_2_y * ys + cuboid.axis_2_z * zs);
        const float a3_proj = std::abs(cuboid.axis_3_x * xs + cuboid.axis_3_y * ys + cuboid.axis_3_z * zs);

        const float a1 = std::max(0.0f, a1_proj - cuboid.axis_1_r);
        const float a2 = std::max(0.0f, a2_proj - cuboid.axis_2_r);
        const float a3 = std::max(0.0f, a3_proj - cuboid.axis_3_r);

        // Squared distance from sphere center to nearest point on cuboid
        const float dist_sq = a1 * a1 + a2 * a2 + a3 * a3;
        
        // Collision if distance is less than radius
        return dist_sq <= rsq;
    }

    bool isWorkspaceCollision(float wx, float wy, float wz) const {

        // check boundary violations
        if(_boundary.size() == 4) {
             if (wx < _boundary[0] || wx > _boundary[1] ||
                wy < _boundary[2] || wy > _boundary[1]) {
                return true;
            }
        } else if(_boundary.size() == 6) {
            if (wx < _boundary[0] || wx > _boundary[3] ||
                wy < _boundary[1] || wy > _boundary[4] ||
                wz < _boundary[2] || wz > _boundary[5]) {
                    return true;
            }
        }

        const float max_extent = std::sqrt(wx * wx + wy * wy + wz * wz) + _robotRadius;
        
        // Check collision with all cuboid obstacles
        for (const auto& cuboid : env_.cuboids) {
            const float diff = cuboid.min_distance - max_extent;
            
            // Early termination: if min_distance > max_extent, no further checks needed
            if (diff > 0.0f) {
                break;
            }
            
            if (checkSphereCuboidCollision(cuboid, wx, wy, wz, _robotRadius)) {
                return true;
            }
        }
        
        return false;
    }
};

VampCollisionChecker::VampCollisionChecker(const ParamPtr &pm)
    : BaseCollisionChecker(pm) {
    pimpl_ = std::make_unique<Impl>(pm);
}

VampCollisionChecker::~VampCollisionChecker() = default;

bool VampCollisionChecker::isCollision(const std::vector<Eigen::VectorXd> &trajectory) const {
    if (trajectory.empty()) {
        return false;
    }

    // Check each point in the trajectory
    for (const auto& state : trajectory) {
        const float wx = static_cast<float>(state(0));
        const float wy = static_cast<float>(state(1));
        const float wz = state.size() > 2 ? static_cast<float>(state(2)) : 0.0f;
        
        if (pimpl_->isWorkspaceCollision(wx, wy, wz)) {
            return true;
        }
    }
    
    return false; // No collision detected
}

} // namespace vamp
