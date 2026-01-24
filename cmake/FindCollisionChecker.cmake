# FindCollisionChecker.cmake
# Finds collision checker libraries built from libcollisionchecker project
#
# This module defines:
#  CollisionChecker_FOUND - True if at least one collision checker library is found
#  CollisionChecker_LIBRARIES - List of all found collision checker libraries
#  CollisionChecker_INCLUDE_DIRS - List of all include directories
#  CollisionChecker_<COMPONENT>_FOUND - True if specific component is found
#  CollisionChecker_<COMPONENT>_LIBRARY - Path to specific component library
#  CollisionChecker_<COMPONENT>_INCLUDE_DIR - Include directory for specific component
#
# Components: OctreeCollisionChecker, QuadtreeCollisionChecker, OccupancyMap, FCL, VampCollisionChecker
#
# Usage:
#  find_package(CollisionChecker REQUIRED COMPONENTS OctreeCollisionChecker QuadtreeCollisionChecker)
#  or
#  find_package(CollisionChecker REQUIRED) # finds all available libraries

# Define the base search path (you may need to adjust this)
set(CollisionChecker_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../libcollisionchecker" CACHE PATH "Root directory of libcollisionchecker")

# Initialize variables
set(CollisionChecker_FOUND FALSE)
set(CollisionChecker_LIBRARIES)
set(CollisionChecker_INCLUDE_DIRS)

# Define available components
set(CollisionChecker_COMPONENTS
        OctreeCollisionChecker
        QuadtreeCollisionChecker
        OccupancyMap
        FCL
        VampCollisionChecker
)

# Component to subdirectory mapping for includes
set(OctreeCollisionChecker_SUBDIR "octree")
set(QuadtreeCollisionChecker_SUBDIR "quadtree")
set(OccupancyMap_SUBDIR "occupancy")
set(FCL_SUBDIR "FCL")
set(VampCollisionChecker_SUBDIR "VAMP")

# If no components specified, try to find all
if(NOT CollisionChecker_FIND_COMPONENTS)
    set(CollisionChecker_FIND_COMPONENTS ${CollisionChecker_COMPONENTS})
endif()

# Search for each requested component
foreach(component ${CollisionChecker_FIND_COMPONENTS})
    # Check if component is valid
    list(FIND CollisionChecker_COMPONENTS ${component} component_index)
    if(component_index EQUAL -1)
        if(CollisionChecker_FIND_REQUIRED_${component})
            message(FATAL_ERROR "Unknown CollisionChecker component: ${component}")
        else()
            message(WARNING "Unknown CollisionChecker component: ${component}")
        endif()
        continue()
    endif()

    # Find the library
    find_library(CollisionChecker_${component}_LIBRARY
            NAMES ${component}
            HINTS
            ${CollisionChecker_ROOT_DIR}/build
            ${CollisionChecker_ROOT_DIR}/lib
            ${CMAKE_INSTALL_PREFIX}/lib
            PATH_SUFFIXES
            lib
    )

    # Find the include directory
    set(component_subdir ${${component}_SUBDIR})
    find_path(CollisionChecker_${component}_INCLUDE_DIR
            NAMES ${component}.h ${component}.hpp
            HINTS
            ${CollisionChecker_ROOT_DIR}/include/${component_subdir}
            ${CollisionChecker_ROOT_DIR}/include
            ${CMAKE_INSTALL_PREFIX}/include
            PATH_SUFFIXES
            ${component_subdir}
            include/${component_subdir}
    )

    # Handle REQUIRED flag for this component
    if(CollisionChecker_${component}_LIBRARY AND CollisionChecker_${component}_INCLUDE_DIR)
        set(CollisionChecker_${component}_FOUND TRUE)
        list(APPEND CollisionChecker_LIBRARIES ${CollisionChecker_${component}_LIBRARY})
        list(APPEND CollisionChecker_INCLUDE_DIRS ${CollisionChecker_${component}_INCLUDE_DIR})

        # Also add the parent include directory
        get_filename_component(parent_include_dir ${CollisionChecker_${component}_INCLUDE_DIR} DIRECTORY)
        list(APPEND CollisionChecker_INCLUDE_DIRS ${parent_include_dir})

        if(NOT CollisionChecker_FIND_QUIETLY)
            message(STATUS "Found ${component}: ${CollisionChecker_${component}_LIBRARY}")
        endif()
    else()
        set(CollisionChecker_${component}_FOUND FALSE)
        if(CollisionChecker_FIND_REQUIRED_${component})
            message(FATAL_ERROR "Could not find required CollisionChecker component: ${component}")
        elseif(NOT CollisionChecker_FIND_QUIETLY)
            message(STATUS "Could not find CollisionChecker component: ${component}")
        endif()
    endif()
endforeach()

# Remove duplicate include directories
if(CollisionChecker_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES CollisionChecker_INCLUDE_DIRS)
endif()

# Set the overall FOUND flag
if(CollisionChecker_LIBRARIES)
    set(CollisionChecker_FOUND TRUE)
endif()

# Handle standard arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CollisionChecker
        REQUIRED_VARS CollisionChecker_LIBRARIES CollisionChecker_INCLUDE_DIRS
        HANDLE_COMPONENTS
)

# Mark variables as advanced
mark_as_advanced(
        CollisionChecker_ROOT_DIR
        CollisionChecker_LIBRARIES
        CollisionChecker_INCLUDE_DIRS
)

foreach(component ${CollisionChecker_COMPONENTS})
    mark_as_advanced(
            CollisionChecker_${component}_LIBRARY
            CollisionChecker_${component}_INCLUDE_DIR
    )
endforeach()

# Print summary if not quiet
if(CollisionChecker_FOUND AND NOT CollisionChecker_FIND_QUIETLY)
    message(STATUS "CollisionChecker found:")
    message(STATUS "  Libraries: ${CollisionChecker_LIBRARIES}")
    message(STATUS "  Include dirs: ${CollisionChecker_INCLUDE_DIRS}")
endif()