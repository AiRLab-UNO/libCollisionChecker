//
// Created by airlab on 1/23/26.
//

#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H

#include <vector>
#include <array>
#include <memory>
#include <Eigen/Dense>
#include "ParamManager.h"
class BaseCollisionChecker;
using CCPtr = std::shared_ptr<BaseCollisionChecker>;

class BaseCollisionChecker: public std::enable_shared_from_this<BaseCollisionChecker>{
public:
    BaseCollisionChecker(const ParamPtr& pm):pm_(pm){

    }
    virtual bool isCollision(const std::vector<Eigen::VectorXd>&trajectory) const = 0;
    CCPtr getSharedPtr(){
        return shared_from_this();
    }

protected:
    ParamPtr pm_;


};

namespace quadtree {
    /**
    * @brief Octree-based collision checker for robot trajectories.
    *
    * This class provides a minimal public interface, hiding implementation details
    * (such as the Octree template instantiation) using the Pimpl idiom.
    */
    class QuadtreeCollisionChecker : public BaseCollisionChecker {
    public:
        /**
         * @brief Construct a new Octree Collision Checker.
         * @param pm Shared pointer to application parameters (boundary, obstacles, etc.)
         */
        explicit QuadtreeCollisionChecker(const ParamPtr &pm);


        /**
         * @brief Check if a trajectory results in a collision.
         * @param trajectory A sequence of 3D points (Eigen::VectorXd).
         * @return true if any point in the trajectory is in collision with obstacles or boundary.
         * @return false if the trajectory is collision-free.
         */
        bool isCollision(const std::vector<Eigen::VectorXd> &trajectory) const override;
    private:
        struct Impl;
        std::unique_ptr<Impl> pimpl_;
    };
}


class OctreeCollisionChecker : public BaseCollisionChecker {
public:
    /**
     * @brief Construct a new Octree Collision Checker.
     * @param pm Shared pointer to application parameters (boundary, obstacles, etc.)
     */
    explicit OctreeCollisionChecker(const ParamPtr &pm);

    /**
     * @brief Destroy the Octree Collision Checker.
     */
    ~OctreeCollisionChecker();

    /**
     * @brief Check if a trajectory results in a collision.
     * @param trajectory A sequence of 3D points (Eigen::VectorXd).
     * @return true if any point in the trajectory is in collision with obstacles or boundary.
     * @return false if the trajectory is collision-free.
     */
    bool isCollision(const std::vector<Eigen::VectorXd> &trajectory) const override;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

namespace occupancy {

    /**
     * @brief Octree-based collision checker for robot trajectories.
     *
     * This class provides a minimal public interface, hiding implementation details
     * (such as the Octree template instantiation) using the Pimpl idiom.
     */
    class OccupancyMap : public BaseCollisionChecker {
    public:
        /**
         * @brief Construct a new Octree Collision Checker.
         * @param pm Shared pointer to application parameters (boundary, obstacles, etc.)
         */
        explicit OccupancyMap(const ParamPtr &pm);

        /**
         * @brief Destroy the Octree Collision Checker.
         */
        ~OccupancyMap();

        /**
         * @brief Check if a trajectory results in a collision.
         * @param trajectory A sequence of 3D points (Eigen::VectorXd).
         * @return true if any point in the trajectory is in collision with obstacles or boundary.
         * @return false if the trajectory is collision-free.
         */
        bool isCollision(const std::vector<Eigen::VectorXd> &trajectory) const override;
    private:
        struct Impl;
        std::unique_ptr<Impl> pimpl_;
    };
} // occupancy

namespace fcl {
    /**
     * @brief Octree-based collision checker for robot trajectories.
     *
     * This class provides a minimal public interface, hiding implementation details
     * (such as the Octree template instantiation) using the Pimpl idiom.
     */
    class FCL : public BaseCollisionChecker {
    public:
        /**
         * @brief Construct a new Octree Collision Checker.
         * @param pm Shared pointer to application parameters (boundary, obstacles, etc.)
         */
        explicit FCL(const ParamPtr &pm);

        /**
         * @brief Destroy the Octree Collision Checker.
         */
        ~FCL();

        /**
         * @brief Check if a trajectory results in a collision.
         * @param trajectory A sequence of 3D points (Eigen::VectorXd).
         * @return true if any point in the trajectory is in collision with obstacles or boundary.
         * @return false if the trajectory is collision-free.
         */
        bool isCollision(const std::vector<Eigen::VectorXd> &trajectory) const override;
    private:
        struct Impl;
        std::unique_ptr<Impl> pimpl_;
    };

} // fcl

#endif //COLLISIONCHECKER_H
