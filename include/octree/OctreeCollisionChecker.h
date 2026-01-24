#ifndef OCTREECOLLISIONCHECKER_H
#define OCTREECOLLISIONCHECKER_H

#include "common/BaseCollisionChecker.h"
#include <memory>
#include <vector>
#include <Eigen/Dense>

/**
 * @brief Octree-based collision checker for robot trajectories.
 * 
 * This class provides a minimal public interface, hiding implementation details
 * (such as the Octree template instantiation) using the Pimpl idiom.
 */
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
    ~OctreeCollisionChecker() = default;

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

#endif //OCTREECOLLISIONCHECKER_H
