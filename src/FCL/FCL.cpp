//
// Created by airlab on 1/23/26.
//


#include "CollisionChecker.hh"
#include <fcl/fcl.h>
namespace fcl {
    struct FCL::Impl {
        Impl(const ParamPtr& pm) {
            float obsLen = pm->get_param<float>("obstacle_length");
            auto obsList = pm->get_ndarray<float>("obstacles");
            _robotRadius = pm->get_param<float>("robot_radius");
            _boundary = pm->get_param<std::vector<float>>("boundary");
            
            _robotGeom = std::make_shared<fcl::Sphere<float>>(_robotRadius);
            
            // Initialize the collision manager first
            _manager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();

            if (pm->has_param("triangles")) {
                setTriangles(pm->get_triangles());
            }

            if (!obsList.empty()) {
                obsLen *= 2.0f;
                auto geom = std::make_shared<fcl::Box<float>>(obsLen, obsLen, obsLen);

                for (const auto& o : obsList) {
                    fcl::Transform3f pose = fcl::Transform3f::Identity();
                    pose.linear() = Eigen::Quaternionf::Identity().matrix();
                    _is3Dobs = o.size() > 2;
                    
                    Eigen::Vector3f translation = Eigen::Vector3f::Zero();
                    for (size_t j = 0; j < o.size() && j < 3; j++)
                        translation[j] = o[j];
                    
                    pose.translation() = translation;
                    _obs_list.emplace_back(std::make_shared<fcl::CollisionObject<float>>(geom, pose));
                }

                std::vector<fcl::CollisionObject<float>*> raw_obs_list;
                raw_obs_list.reserve(_obs_list.size());
                for (auto& obj : _obs_list) {
                    raw_obs_list.push_back(obj.get());
                }
                _manager->registerObjects(raw_obs_list);
                _manager->setup();
            }
        }

        virtual ~Impl() = default;

        bool isCollision(const std::vector<Eigen::VectorXd>& trajectory) const {
            if (_obs_list.empty()) {
                return false;
            }

            if (_boundary.size() == 4) {
                for (const auto& state : trajectory) {
                    if (state(0) < _boundary[0] || state(0) > _boundary[1] ||
                        state(1) < _boundary[2] || state(1) > _boundary[3]) {
                        return true;
                    }
                }
            }

            std::vector<std::shared_ptr<fcl::CollisionObject<float>>> trajectory_objects;
            trajectory_objects.reserve(trajectory.size());

            for (const auto& state : trajectory) {
                fcl::Transform3f pose = fcl::Transform3f::Identity();
                pose.linear() = Eigen::Quaternionf::Identity().matrix();
                if (_is3Dobs)
                    pose.translation() = Eigen::Vector3f(float(state(0)), float(state(1)), float(state(2)));
                else
                    pose.translation() = Eigen::Vector3f(float(state(0)), float(state(1)), 0.0f);
                trajectory_objects.emplace_back(std::make_shared<fcl::CollisionObject<float>>(_robotGeom, pose));
            }

            fcl::DynamicAABBTreeCollisionManager<float> trajectory_manager;
            std::vector<fcl::CollisionObject<float>*> raw_trajectory_objects;
            raw_trajectory_objects.reserve(trajectory_objects.size());
            for (auto& obj : trajectory_objects) {
                raw_trajectory_objects.push_back(obj.get());
            }
            trajectory_manager.registerObjects(raw_trajectory_objects);
            trajectory_manager.setup();

            fcl::DefaultCollisionData<float> collision_data;
            trajectory_manager.collide(_manager.get(), &collision_data, fcl::DefaultCollisionFunction);

            return collision_data.result.isCollision();
        }

        float minDist(const std::vector<Eigen::VectorXd>& trajectory) const {
            if (_obs_list.empty()) {
                return std::numeric_limits<float>::max();
            }

            std::vector<std::shared_ptr<fcl::CollisionObject<float>>> trajectory_objects;
            trajectory_objects.reserve(trajectory.size());

            for (const auto& state : trajectory) {
                fcl::Transform3f pose = fcl::Transform3f::Identity();
                pose.linear() = Eigen::Quaternionf::Identity().matrix();
                if (_is3Dobs)
                    pose.translation() = Eigen::Vector3f(float(state(0)), float(state(1)), float(state(2)));
                else
                    pose.translation() = Eigen::Vector3f(float(state(0)), float(state(1)), 0.0f);
                trajectory_objects.emplace_back(std::make_shared<fcl::CollisionObject<float>>(_robotGeom, pose));
            }

            fcl::DynamicAABBTreeCollisionManager<float> trajectory_manager;
            std::vector<fcl::CollisionObject<float>*> raw_trajectory_objects;
            raw_trajectory_objects.reserve(trajectory_objects.size());
            for (auto& obj : trajectory_objects) {
                raw_trajectory_objects.push_back(obj.get());
            }
            trajectory_manager.registerObjects(raw_trajectory_objects);
            trajectory_manager.setup();

            fcl::DefaultDistanceData<float> distance_data;
            trajectory_manager.distance(_manager.get(), &distance_data, fcl::DefaultDistanceFunction);

            return distance_data.result.min_distance;
        }

        void setTriangles(const std::vector<Triangle2D>& triangles) {
            _manager->clear();
            _obs_list.clear();
            _obs_list.reserve(triangles.size());

            for (const auto& triangle : triangles) {
                auto geom = std::make_shared<fcl::BVHModel<fcl::OBBRSS<float>>>();
                geom->beginModel();
                fcl::Vector3<float> v0(triangle.vertices[0][0], triangle.vertices[0][1], 0.0f);
                fcl::Vector3<float> v1(triangle.vertices[1][0], triangle.vertices[1][1], 0.0f);
                fcl::Vector3<float> v2(triangle.vertices[2][0], triangle.vertices[2][1], 0.0f);
                geom->addTriangle(v0, v1, v2);
                geom->endModel();

                fcl::Transform3<float> pose = fcl::Transform3<float>::Identity();
                _obs_list.emplace_back(std::make_shared<fcl::CollisionObject<float>>(geom, pose));
            }

            if (!_obs_list.empty()) {
                std::vector<fcl::CollisionObject<float>*> raw_obs_list;
                raw_obs_list.reserve(_obs_list.size());
                for (auto& obj : _obs_list) {
                    raw_obs_list.push_back(obj.get());
                }
                _manager->registerObjects(raw_obs_list);
                _manager->setup();
            }
        }

    private:
        float _robotRadius;
        std::shared_ptr<fcl::Sphere<float>> _robotGeom;
        std::vector<std::shared_ptr<fcl::CollisionObject<float>>> _obs_list;
        std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> _manager;
        std::vector<float> _boundary;
        bool _is3Dobs = false;

    };


    FCL::FCL(const ParamPtr &pm):BaseCollisionChecker(pm) {
        pimpl_ = std::make_unique<Impl>(pm);
    }

    FCL::~FCL() {
    }

    bool FCL::isCollision(const std::vector<Eigen::VectorXd> &trajectory) const {
        return pimpl_->isCollision(trajectory);
    }
} // fcl