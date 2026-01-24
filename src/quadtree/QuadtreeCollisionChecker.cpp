//
// Created by airlab on 1/23/26.
//

#include "quadtree/QuadtreeCollisionChecker.h"
#include "quadtree/olcUTIL_QuadTree.h"

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

        float radii = _robotRadius;

        for(int i = 0; i < obsList.size(); ++i)
        {
            obstacle obs;
            obs.id = i + 1;
            obs.type = 2;
            obs.x = obsList[i][0];
            obs.y = obsList[i][1];
            obs.width = radii;
            obs.height = radii;
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
        robot.x = wx;
        robot.y = wy;
        robot.width = _robotRadius ;
        robot.height = _robotRadius;
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

} // occupancy