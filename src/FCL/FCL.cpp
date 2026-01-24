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
            // Initialize the collision manager first
            _manager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();

            if (pm->has_param("triangles")) {
                setTriangles(pm->get_triangles());
            }

            if (!obsList.empty()) {
                obsLen *= 2.0f;
                auto geom = std::make_shared<fcl::Box<float>>(obsLen, obsLen, obsLen);
                // _obs_list.reserve(obsList.size());

                for (const auto& o : obsList) {
                    fcl::Transform3f pose = fcl::Transform3f::Identity();
                    pose.linear() = Eigen::Quaternionf::Identity().matrix();
                    _is3Dobs = o.size() > 2;
                    std::vector<float>temp(3, 0.0);
                    for (int j = 0; j < o.size() || j < temp.size(); j++)
                        temp[j] = o[j];
                    pose.translation() = Eigen::Vector3f(temp[0], temp[1], temp[2]);
                    _obs_list.emplace_back(new fcl::CollisionObject<float>(geom, pose));
                }

                _manager->registerObjects(_obs_list);
                _manager->setup();
            } else {
                // DEBUG("Warning: No obstacles provided to CollisionChecker.");
            }
        }

        virtual ~Impl() {
            for (auto* obj : _obs_list) {
                delete obj;
            }
        }
        bool isCollision(const std::vector<Eigen::VectorXd>&trajectory) const {
            // Check if we have any obstacles registered
            if (_obs_list.empty()) {
                // DEBUG("Warning: No obstacles registered for collision checking");
                return false;
            }

            if(_boundary.size() == 4){
                // Check boundary violations [xmin, xmax, ymin, ymax]
                for(const auto& state : trajectory){
                    if(state(0) < _boundary[0] || state(0) > _boundary[1] ||
                       state(1) < _boundary[2] || state(1) > _boundary[3]){
                        return true;
                       }
                }
            }

            auto geom = std::make_shared<fcl::Sphere<float>>(_robotRadius);
            std::vector<fcl::CollisionObject<float>*> trajectory_objects;
            trajectory_objects.reserve(trajectory.size());

            for (const auto& state : trajectory) {
                fcl::Transform3f pose = fcl::Transform3f::Identity();
                pose.linear() = Eigen::Quaternionf::Identity().matrix();
                if (_is3Dobs)
                    pose.translation() = Eigen::Vector3f(float(state(0)), float(state(1)), float(state(2))); // Changed to 0.0f for 3D
                else
                    pose.translation() = Eigen::Vector3f(float(state(0)), float(state(1)), 0.0f); // Changed to 0.0f for 2D
                trajectory_objects.emplace_back(new fcl::CollisionObject<float>(geom, pose));
            }

            fcl::DynamicAABBTreeCollisionManager<float> trajectory_manager;
            trajectory_manager.registerObjects(trajectory_objects);
            trajectory_manager.setup();

            fcl::DefaultCollisionData<float> collision_data;
            trajectory_manager.collide(_manager.get(), &collision_data, fcl::DefaultCollisionFunction);

            bool has_collision = collision_data.result.isCollision();

            // Clean up trajectory objects
            for (auto* obj : trajectory_objects) {
                delete obj;
            }

            return has_collision;

        }
        float minDist(const std::vector<Eigen::VectorXd>& trajectory) const {
            // Check if we have any obstacles registered
            if (_obs_list.empty()) {
                // DEBUG("Warning: No obstacles registered for distance checking");
                return std::numeric_limits<float>::max();
            }

            auto geom = std::make_shared<fcl::Sphere<float>>(_robotRadius);
            std::vector<fcl::CollisionObject<float>*> trajectory_objects;
            trajectory_objects.reserve(trajectory.size());

            for (const auto& state : trajectory) {
                fcl::Transform3f pose = fcl::Transform3f::Identity();
                pose.linear() = Eigen::Quaternionf::Identity().matrix();
                pose.translation() = Eigen::Vector3f(float(state(0)), float(state(1)), 0.0f); // Changed to 0.0f for 2D
                trajectory_objects.emplace_back(new fcl::CollisionObject<float>(geom, pose));
            }

            fcl::DynamicAABBTreeCollisionManager<float> trajectory_manager;
            trajectory_manager.registerObjects(trajectory_objects);
            trajectory_manager.setup();

            fcl::DefaultDistanceData<float> distance_data;
            trajectory_manager.distance(_manager.get(), &distance_data, fcl::DefaultDistanceFunction);

            float min_distance = distance_data.result.min_distance;

            // Clean up trajectory objects
            for (auto* obj : trajectory_objects) {
                delete obj;
            }

            return min_distance;
        }
        void setTriangles(const std::vector<Triangle2D>& triangles) {
            // DEBUG("Setting triangles in CollisionChecker " << triangles.size() << " triangles");

            // Clear existing objects from manager
            _manager->clear();

            // Clean up existing collision objects
            for (auto* obj : _obs_list) {
                delete obj;
            }
            _obs_list.clear();

            // Reserve space for efficiency
            _obs_list.reserve(triangles.size());

            for (const auto& triangle : triangles) {
                // Create a BVH model for the triangle mesh
                auto geom = std::make_shared<fcl::BVHModel<fcl::OBBRSS<float>>>();

                // Begin model construction
                geom->beginModel();

                // Use triangles in their original coordinates (no centering needed)
                fcl::Vector3<float> v0(triangle.vertices[0][0], triangle.vertices[0][1], 0.0f);
                fcl::Vector3<float> v1(triangle.vertices[1][0], triangle.vertices[1][1], 0.0f);
                fcl::Vector3<float> v2(triangle.vertices[2][0], triangle.vertices[2][1], 0.0f);

                // Add the triangle using the vertex positions
                geom->addTriangle(v0, v1, v2);

                // End model construction
                geom->endModel();

                // Create identity transform (no translation needed since we use original coordinates)
                fcl::Transform3<float> pose = fcl::Transform3<float>::Identity();

                // Create collision object and add to list
                _obs_list.emplace_back(new fcl::CollisionObject<float>(geom, pose));
            }

            // Register objects with the collision manager
            if (_obs_list.empty()) {
                // DEBUG("Warning: No triangles provided to CollisionChecker.");
            } else {
                _manager->registerObjects(_obs_list);
                _manager->setup();
                // DEBUG("Registered " + std::to_string(triangles.size()) + " triangles with collision manager.");
            }
        }

    private:
        float _robotRadius;
        std::vector<fcl::CollisionObject<float>*> _obs_list;
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