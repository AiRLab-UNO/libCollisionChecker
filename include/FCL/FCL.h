//
// Created by airlab on 1/23/26.
//

#ifndef FCL_H
#define FCL_H
#include "common/BaseCollisionChecker.h"
#include <memory>
#include <vector>
#include <Eigen/Dense>

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

#endif //FCL_H
