cmake_minimum_required(VERSION 2.8.3)
project(octomap_world)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") 

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/octomap_world.cc
  src/octomap_manager.cc
)

############
# BINARIES #
############
cs_add_executable(octomap_manager
  src/octomap_manager_node.cc
)
target_link_libraries(octomap_manager ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
