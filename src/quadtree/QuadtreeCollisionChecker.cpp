//
// Created by airlab on 1/23/26.
//


#include "CollisionChecker.hh"
#include "quadtree/olcUTIL_QuadTree.h"
#include "common/FindCollisionChecker.h"
namespace quadtree {

struct QuadtreeCollisionChecker::Impl {

    Impl(const ParamPtr& pm) {
        _robotRadius = pm->get_param<float>("robot_radius");
        auto boundary = pm->get_param<std::vector<float>>("boundary");
        auto obsList = pm->get_ndarray<float>("obstacles");

        _x_min = boundary[0];
        _x_max = boundary[1];
        _y_min = boundary[2];
        _y_max = boundary[3];
        initialize_manager(pm);
    }

    void initialize_manager(const ParamPtr& pm){
        // Use FindCollisionChecker to determine the type of obstacles and load them accordingly
        FindCollisionChecker finder(pm);
        auto plan = finder.get_plan();
        auto checkers = finder.available_checkers(plan);
        // `FCL` is also the name of this class (fcl::FCL). Use the global-scope enum value to avoid
        // colliding with the class name inside namespace fcl.
        if(std::count(checkers.begin(), checkers.end(), collision_checker_type::QUADTREE) == 0) {
            std::cerr << "QUADTREE is not an available checker for the given environment." << std::endl;
            exit(1);
        }

        if(plan[0] != BOX) {
            std::cerr << "QUADTREE does not support obstacle type. (make sure obs_type is BOX)" << std::endl;
            exit(1);
        }

        auto obsLen = pm->get_param<float>("obstacle_length");
        obsLen *= 2.0f; // Convert half-length to full length for FCL box geometry
        for(auto& box : pm->get_ndarray<float>("obstacles")) {
            obstacle obs;
            switch (plan[2]) {
                case RECTANGLE:
                    obs.type = 2;
                    obs.x = box[0] - box[2] / 2.0f; // Center X coordinate of the rectangle
                    obs.y = box[1] - box[3] / 2.0f; // Center Y coordinate of the rectangle
                    obs.width = box[2];
                    obs.height = box[3];
                    break;
                case SQUARE:
                    obs.type = 2;
                    obs.x = box[0]; // Center X coordinate of the square
                    obs.y = box[1]; // Center Y coordinate of the square
                    obs.width = obsLen;
                    obs.height = obsLen;
                    break;
                default:
                    std::cerr << "Unknown shape type in plan: " << plan[2] << std::endl;
            }

            olc::utils::geom2d::rect<float> rect{{obs.x, obs.y}, {obs.width, obs.height}};
            obstacles_.insert(obs, rect);
        }

    }


    bool isWorkspaceCollision(float wx, float wy) const {
        if(wx < _x_min || wx > _x_max || wy < _y_min || wy > _y_max)
            return true;
        obstacle robot;
        robot.id = 0;
        robot.type = 2;
        robot.x = wx - _robotRadius;  // Center X coordinate of the robot
        robot.y = wy - _robotRadius;  // Center Y coordinate of the robot
        robot.width = _robotRadius * 2.0f;
        robot.height = _robotRadius * 2.0f;

        olc::utils::geom2d::rect<float> rect{{robot.x, robot.y}, {robot.width, robot.height}};

        auto potential_collisions = obstacles_.search(rect);
        if(!potential_collisions.empty())
        {
            for(auto& it: potential_collisions)
            {
                return true; // collision detected
            }
        }
        return false;
    }

private:
    float _robotRadius;
    float _x_min, _x_max, _y_min, _y_max;
    struct obstacle{
        int id;
        int type;
        float x;
        float y;
        float width;
        float height;
    };
    olc::utils::QuadTreeContainer<obstacle> obstacles_;

};

QuadtreeCollisionChecker::QuadtreeCollisionChecker(const ParamPtr &pm)
    : BaseCollisionChecker(pm) {
    pimpl_ = std::make_unique<Impl>(pm);
}

QuadtreeCollisionChecker::~QuadtreeCollisionChecker() = default;


bool QuadtreeCollisionChecker::isCollision(const std::vector<Eigen::VectorXd> &trajectory) const {
    for (int j = trajectory.size(); j-- > 0;) {
        auto state = trajectory[j];
        float wx = state(0);
        float wy = state(1);
        if(pimpl_->isWorkspaceCollision(wx, wy))
        {
            return true;
        }
    }
    return false;
}

} // quadtree