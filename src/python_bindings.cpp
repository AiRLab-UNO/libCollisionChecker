#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <vector>
#include <Eigen/Dense>
#include "CollisionChecker.hh"

namespace py = pybind11;

// Helper function to convert numpy array to std::vector<Eigen::VectorXd>
std::vector<Eigen::VectorXd> numpy_to_trajectory(py::array_t<double> trajectory_array) {
    py::buffer_info buf = trajectory_array.request();
    
    if (buf.ndim != 2) {
        throw std::runtime_error("Trajectory must be a 2D numpy array (N x DOF)");
    }
    
    size_t num_waypoints = buf.shape[0];
    size_t dof = buf.shape[1];
    
    double* ptr = static_cast<double*>(buf.ptr);
    std::vector<Eigen::VectorXd> trajectory;
    trajectory.reserve(num_waypoints);
    
    for (size_t i = 0; i < num_waypoints; i++) {
        Eigen::VectorXd waypoint(dof);
        for (size_t j = 0; j < dof; j++) {
            waypoint(j) = ptr[i * dof + j];
        }
        trajectory.push_back(waypoint);
    }
    
    return trajectory;
}

// Helper function to convert std::vector<Eigen::VectorXd> to numpy array
py::array_t<double> trajectory_to_numpy(const std::vector<Eigen::VectorXd>& trajectory) {
    if (trajectory.empty()) {
        return py::array_t<double>(std::vector<size_t>{0, 0});
    }
    
    size_t num_waypoints = trajectory.size();
    size_t dof = trajectory[0].size();
    
    // Create array with proper shape
    auto result = py::array_t<double>(std::vector<size_t>{num_waypoints, dof});
    auto buf = result.request();
    double* ptr = static_cast<double*>(buf.ptr);
    
    // Copy data
    for (size_t i = 0; i < num_waypoints; i++) {
        for (size_t j = 0; j < dof; j++) {
            ptr[i * dof + j] = trajectory[i](j);
        }
    }
    
    return result;
}

PYBIND11_MODULE(collision_checker_py, m) {
    m.doc() = "Python bindings for libCollisionChecker with numpy array support";
    
    // Bind BaseCollisionChecker
    py::class_<BaseCollisionChecker, CCPtr>(m, "BaseCollisionChecker")
        .def("is_collision", [](const BaseCollisionChecker& self, py::array_t<double> trajectory) {
            return self.isCollision(numpy_to_trajectory(trajectory));
        }, py::arg("trajectory"), 
        "Check if the given trajectory (numpy array of shape [N, DOF]) is in collision.\n\n"
        "Parameters:\n"
        "    trajectory: numpy array of shape [N, DOF] where N is the number of waypoints\n"
        "                and DOF is the degrees of freedom\n\n"
        "Returns:\n"
        "    bool: True if collision detected, False otherwise")
        
        .def("is_collision_vector", &BaseCollisionChecker::isCollision,
        py::arg("trajectory"),
        "Check collision using a vector of Eigen::VectorXd (advanced usage)");
    
    // Bind QuadtreeCollisionChecker
    py::class_<quadtree::QuadtreeCollisionChecker, BaseCollisionChecker, 
               std::shared_ptr<quadtree::QuadtreeCollisionChecker>>(m, "QuadtreeCollisionChecker")
        .def(py::init<const ParamPtr&>(), py::arg("param_manager"),
        "Create a Quadtree-based 2D collision checker.\n\n"
        "Parameters:\n"
        "    param_manager: Parameter manager object with configuration");
    
    // Bind OctreeCollisionChecker
    py::class_<octree::OctreeCollisionChecker, BaseCollisionChecker, 
               std::shared_ptr<octree::OctreeCollisionChecker>>(m, "OctreeCollisionChecker")
        .def(py::init<const ParamPtr&>(), py::arg("param_manager"),
        "Create an Octree-based 3D collision checker.\n\n"
        "Parameters:\n"
        "    param_manager: Parameter manager object with configuration");
    
    // Bind OccupancyMap
    py::class_<occupancy::OccupancyMap, BaseCollisionChecker, 
               std::shared_ptr<occupancy::OccupancyMap>>(m, "OccupancyMap")
        .def(py::init<const ParamPtr&>(), py::arg("param_manager"),
        "Create an occupancy map-based 2D collision checker.\n\n"
        "Parameters:\n"
        "    param_manager: Parameter manager object with configuration");
    
    // Bind FCL
    py::class_<fcl::FCL, BaseCollisionChecker, 
               std::shared_ptr<fcl::FCL>>(m, "FCL")
        .def(py::init<const ParamPtr&>(), py::arg("param_manager"),
        "Create an FCL (Flexible Collision Library) based collision checker.\n"
        "Supports both 2D and 3D collision checking with triangle meshes.\n\n"
        "Parameters:\n"
        "    param_manager: Parameter manager object with configuration");
    
    // Bind VampCollisionChecker
    py::class_<vamp::VampCollisionChecker, BaseCollisionChecker, 
               std::shared_ptr<vamp::VampCollisionChecker>>(m, "VampCollisionChecker")
        .def(py::init<const ParamPtr&>(), py::arg("param_manager"),
        "Create a VAMP (Vectorized Approximation of Minkowski Product) collision checker.\n\n"
        "Parameters:\n"
        "    param_manager: Parameter manager object with configuration");
    
    // Bind param_manager
    py::class_<param_manager, ParamPtr>(m, "ParamManager")
        .def(py::init<const std::string&>(), py::arg("yaml_file"),
        "Create a parameter manager from a YAML configuration file.\n\n"
        "Parameters:\n"
        "    yaml_file: Path to YAML configuration file");
    
    // Utility functions
    m.def("numpy_to_trajectory", &numpy_to_trajectory, py::arg("trajectory_array"),
          "Convert a numpy array (N x DOF) to a vector of Eigen::VectorXd");
    
    m.def("trajectory_to_numpy", &trajectory_to_numpy, py::arg("trajectory"),
          "Convert a vector of Eigen::VectorXd to a numpy array (N x DOF)");
    
    // Version info
    m.attr("__version__") = "1.0.0";
}
