cmake_minimum_required(VERSION 2.8.3)
project(volumetric_map_base)

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
  src/world_base.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
