# Locate the g2o libraries
# A general framework for graph optimization.
#
# This module defines
# G2O_FOUND, if false, do not try to link against g2o
# G2O_LIBRARIES, path to the libg2o
# G2O_INCLUDE_DIR, where to find the g2o header files

# Modified from https://github.com/RainerKuemmerle/g2o/blob/master/cmake_modules/FindG2O.cmake 
# Reference: https://github.com/rpavlik/cmake-modules/blob/main/module-docs/Example-FindMyPackage-UsingImportedTargets.cmake 


include(FindPackageHandleStandardArgs)

find_path(G2O_LIB_DIR libg2o_core.so
  ${G2O_ROOT}/lib
  $ENV{G2O_ROOT}/lib
  NO_DEFAULT_PATH
  )

#message(STATUS "found G2O_LIB_DIR: ${G2O_LIB_DIR}")

# Find the header files

find_path(G2O_INCLUDE_DIR g2o/core/base_vertex.h
  ${G2O_ROOT}/include
  $ENV{G2O_ROOT}/include
  $ENV{G2O_ROOT}
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
  )

# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.

macro(FIND_G2O_LIBRARY MYLIBRARY MYLIBRARYNAME)

  find_library("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS
    ${G2O_ROOT}/lib/Debug
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Debug
    $ENV{G2O_ROOT}/lib
    NO_DEFAULT_PATH
    )

  find_library("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )
  
  find_library(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
    PATHS
    ${G2O_ROOT}/lib/Release
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Release
    $ENV{G2O_ROOT}/lib
    NO_DEFAULT_PATH
    )

  find_library(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )

  if(${MYLIBRARY})
    add_library(g2o_${MYLIBRARYNAME} UNKNOWN IMPORTED GLOBAL)
    set_target_properties(g2o_${MYLIBRARYNAME} PROPERTIES 
      IMPORTED_LOCATION ${${MYLIBRARY}}
      IMPORTED_NO_SONAME ON)
    # https://stackoverflow.com/questions/68164903/cmake-to-link-external-library-with-import-soname-ro-import-location
    #set_property(TARGET g2o_${MYLIBRARYNAME} PROPERTY IMPORTED_NO_SONAME TRUE)
    #message(STATUS "imported location for g2o_${MYLIBRARYNAME}: ${${MYLIBRARY}}")
  endif()

  if(NOT ${MYLIBRARY}_DEBUG)
    if(MYLIBRARY)
      set(${MYLIBRARY}_DEBUG ${MYLIBRARY})
    endif(MYLIBRARY)
  endif(NOT ${MYLIBRARY}_DEBUG)
  
endmacro(FIND_G2O_LIBRARY LIBRARY LIBRARYNAME)

# Find the core elements
FIND_G2O_LIBRARY(G2O_STUFF_LIBRARY stuff)
FIND_G2O_LIBRARY(G2O_CORE_LIBRARY core)

# Find the CLI library
FIND_G2O_LIBRARY(G2O_CLI_LIBRARY cli)

# Find the pluggable solvers
FIND_G2O_LIBRARY(G2O_SOLVER_CHOLMOD solver_cholmod)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE solver_csparse)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE_EXTENSION csparse_extension)
FIND_G2O_LIBRARY(G2O_SOLVER_DENSE solver_dense)
FIND_G2O_LIBRARY(G2O_SOLVER_PCG solver_pcg)
FIND_G2O_LIBRARY(G2O_SOLVER_SLAM2D_LINEAR solver_slam2d_linear)
FIND_G2O_LIBRARY(G2O_SOLVER_STRUCTURE_ONLY solver_structure_only)
FIND_G2O_LIBRARY(G2O_SOLVER_EIGEN solver_eigen)

# Find the predefined types
FIND_G2O_LIBRARY(G2O_TYPES_DATA types_data)
FIND_G2O_LIBRARY(G2O_TYPES_ICP types_icp)
FIND_G2O_LIBRARY(G2O_TYPES_SBA types_sba)
FIND_G2O_LIBRARY(G2O_TYPES_SCLAM2D types_sclam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SIM3 types_sim3)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D types_slam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D types_slam3d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D_ADDSON types_slam3d_addons)

# G2O solvers declared found if we found at least one solver
set(G2O_SOLVERS_FOUND "NO")
if(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)
  set(G2O_SOLVERS_FOUND "YES")
endif(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)

# G2O itself declared found if we found the core libraries and at least one solver
set(G2O_FOUND "NO")
if(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
  set(G2O_FOUND "YES")
endif(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)


# adaptation for collecting all together the libs 
if(G2O_FOUND)
    set(G2O_LIBS
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_CLI_LIBRARY}
        ${G2O_SOLVER_CHOLMOD}
        ${G2O_SOLVER_CSPARSE}
        ${G2O_SOLVER_CSPARSE_EXTENSION}
        ${G2O_SOLVER_DENSE}
        ${G2O_SOLVER_PCG}
        ${G2O_SOLVER_SLAM2D_LINEAR}
        ${G2O_SOLVER_STRUCTURE_ONLY}
        ${G2O_SOLVER_EIGEN}
        ${G2O_TYPES_DATA}
        ${G2O_TYPES_ICP}
        ${G2O_TYPES_SBA}
        ${G2O_TYPES_SCLAM2D}
        ${G2O_TYPES_SIM3}
        ${G2O_TYPES_SLAM2D}
        ${G2O_TYPES_SLAM3D}
        ${G2O_TYPES_SLAM3D_ADDSON}
    )
endif()

# if(G2O_FOUND)
#     set(G2O_LIBS
#         g2o_core
#         g2o_stuff
#         g2o_cli
#         g2o_solver_cholmod
#         g2o_solver_csparse
#         g2o_csparse_extension
#         g2o_solver_dense
#         g2o_solver_pcg
#         g2o_solver_slam2d_linear
#         g2o_solver_structure_only
#         g2o_solver_eigen
#         g2o_types_data
#         g2o_types_icp
#         g2o_types_sba
#         g2o_types_sclam2d
#         g2o_types_sim3
#         g2o_types_slam2d
#         g2o_types_slam3d
#         g2o_types_slam3d_addons
#     )
# endif()