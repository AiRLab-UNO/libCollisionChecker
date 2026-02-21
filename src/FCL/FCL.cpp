//
// Created by airlab on 1/23/26.
//


#include "CollisionChecker.hh"
#include "common/FindCollisionChecker.h"
#include <fcl/fcl.h>
#include <algorithm>
namespace fcl {
    struct FCL::Impl {
        Impl(const ParamPtr& pm) {

            _robotRadius = pm->get_param<float>("robot_radius");
            _boundary = pm->get_param<std::vector<float>>("boundary");
            _robotGeom = std::make_shared<fcl::Sphere<float>>(_robotRadius);
            initialize_manager(pm);
        }

        void initialize_manager(const ParamPtr& pm) {
            // Initialize the collision manager first
            _manager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();

            // Use FindCollisionChecker to determine the type of obstacles and load them accordingly
            FindCollisionChecker finder(pm);
            auto plan = finder.get_plan();
            auto checkers = finder.available_checkers(plan);
            // `FCL` is also the name of this class (fcl::FCL). Use the global-scope enum value to avoid
            // colliding with the class name inside namespace fcl.
            if(std::count(checkers.begin(), checkers.end(), collision_checker_type::FCL) == 0) {
                std::cerr << "FCL is not an available checker for the given environment." << std::endl;
                exit(1);
            }


            if(plan[0] == TRIANGLES) {
                setTriangles(pm->get_triangles());
            }
            else if(plan[0] == BOX) {
               auto obsLen = pm->get_param<float>("obstacle_length");
               obsLen *= 2.0f; // Convert half-length to full length for FCL box geometry
               for(auto& box : pm->get_ndarray<float>("obstacles")) {
                   switch (plan[2]) {
                       case RECTANGLE:
                                setBox({box[0], box[1], 0.0f, box[2], box[3], obsLen});
                           break;
                       case SQUARE:
                                setBox({box[0], box[1], 0.0f, obsLen, obsLen, obsLen});
                           break;
                       case CUBE:
                                _is3Dobs = true;
                                setBox({box[0], box[1], box[2], obsLen, obsLen, obsLen});
                           break;
                       case CUBOID:
                                _is3Dobs = true;
                                setBox({box[0], box[1], box[2], box[3], box[4], box[5]});
                           break;
                       default:
                           std::cerr << "Unknown shape type in plan: " << plan[2] << std::endl;
                   }
               }
             }

            // configure the collision manager with the loaded obstacles
            std::vector<fcl::CollisionObject<float>*> raw_obs_list;
            raw_obs_list.reserve(_obs_list.size());
            for (auto& obj : _obs_list) {
                raw_obs_list.push_back(obj.get());
            }
            _manager->registerObjects(raw_obs_list);
            _manager->setup();
        }

        void setBox(const std::array<float, 6>& box){

            fcl::Transform3f pose = fcl::Transform3f::Identity();
            pose.linear() = Eigen::Quaternionf::Identity().matrix();
            auto geom = std::make_shared<fcl::Box<float>>(box[3],box[4], box[5]);
            Eigen::Vector3f translation = Eigen::Vector3f::Zero();
            for (size_t j = 0; j < 3; j++)
                translation[j] = box[j];
            pose.translation() = translation;
            _obs_list.emplace_back(std::make_shared<fcl::CollisionObject<float>>(geom, pose));
        }

        virtual ~Impl() = default;

        bool isCollision(const std::vector<Eigen::VectorXd>& trajectory) const {
            if (_obs_list.empty()) {
                return false;
            }

            if (_boundary.size() == 4) {

            }

            for (const auto& state : trajectory) {
                float wx = static_cast<float>(state(0));
                float wy = static_cast<float>(state(1));
                float wz = static_cast<float>(state(2));
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