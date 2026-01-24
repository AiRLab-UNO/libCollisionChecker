//
// Created by airlab on 1/23/26.
//

#include "vamp/VampCollisionChecker.h"
#include "vamp/robots/sphere.hh"
#include "vamp/collision/factory.hh"
#include "vamp/collision/environment.hh"
namespace vc = vamp::collision;
namespace vf = vamp::collision::factory;

namespace vamp {

struct VampCollisionChecker::Impl {

    vc::Environment<float> env_;

    Impl(const ParamPtr& pm) {
        _robotRadius = pm->get_param<float>("robot_radius");
        auto boundary = pm->get_param<std::vector<float>>("boundary");
        auto obstacles = pm->get_ndarray<float>("obstacles");
        auto obs_len = pm->get_param<float>("obstacle_length");

        for(const auto& obs: obstacles)
        {
            std::vector<float>temp(3, 0.0);
            for (int j = 0; j < obs.size() || j < temp.size(); j++)
                temp[j] = obs[j];
            env_.cuboids.push_back(vc::Cuboid<float>(
                    temp[0], temp[1], temp[2],
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0,
                    obs_len, obs_len, obs_len
            ));
        }
        env_.sort(); // Sort the obstacles
    }


    bool isWorkspaceCollision(float wx, float wy) const {

        return false;
    }

private:
    float _robotRadius;
    float _x_min, _x_max, _y_min, _y_max;

};

VampCollisionChecker::VampCollisionChecker(const ParamPtr &pm)
    : BaseCollisionChecker(pm) {
    pimpl_ = std::make_unique<Impl>(pm);
}


bool VampCollisionChecker::isCollision(const std::vector<Eigen::VectorXd> &trajectory) const {
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
} // vamp