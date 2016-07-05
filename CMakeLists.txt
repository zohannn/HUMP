#=========================================================================
#   Institute:   University of Minho
#                Dept. of Industrial Electronics
#
#   File:        CMakeLists.txt
#
#   Maintainer:  Gianpaolo Gulletta
#   Email:       gianpaologulletta@gmail.com; d6468@dei.uminho.pt
#   Date:        2016 01 12
#
#   Description: Human-like Upper-limbs Motion Library
#
#   Credits:
#==========================================================================

cmake_minimum_required(VERSION 2.8)
#project name
project (HUML)

SET(CMAKE_CXX_FLAGS "-std=c++11")

## Humanoid parts configuration: ##
# 1 means that the part is present and it is taken into account for the generation of the trajectories,
# 0 otherwise.
# The torso and the arms are the only parts assumed for any humanoid robot. In other words, the anthropomorphiism of the system is assumed.
# The characteristics of the torso will be those ones of the humanoid.
# However the following parts may be taken into account when they are present
# set (HEAD 0)
# set (NECK 0)
# set (PELVIS 1)
# set (RIGHT_UPPER_LEG 1)
# set (RIGHT_LOWER_LEG 0)
# set (RIGHT_FOOT 0)
# set (LEFT_UPPER_LEG 1)
# set (LEFT_LOWER_LEG 0)
# set (LEFT_FOOT 0)


## Hand configuration: ##
# HAND=0. Human hand
# HAND=1, Barrett hand
set (HAND 1)


# sources
file(GLOB_RECURSE HPP_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp)
file(GLOB_RECURSE CPP_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

# configuration file
configure_file (
  "${CMAKE_CURRENT_SOURCE_DIR}/include/config.h.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/config.h"
)

##############################################################################
# External libraries
##############################################################################

# AMPL software
# AMPL=0 use wine to run ampl.exe
# AMPL=1 use ampl for linux
set (AMPL 0)

# IPOPT Solver
set(IPOPT_HOME /home/gianpaolo/CoinIpopt)

include_directories(

    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/coin
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/coin/ThirdParty
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/extra

    )

#link_directories(

 #   ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/lib

#)

#if(UNIX)
 #   link_libraries(dl)
  #  link_libraries(gfortran)

#endif(UNIX)

# add the library
add_library(HUML STATIC ${HPP_SOURCES} ${CPP_SOURCES})
# link libraries
# target_link_libraries(HUML libipopt.a)
# target_link_libraries(HUML libcoinmumps.a)
# target_link_libraries(HUML libcoinlapack.a)
# target_link_libraries(HUML libcoinasl.a)
# target_link_libraries(HUML libcoinblas.a)
# target_link_libraries(HUML libcoinmetis.a)
# target_link_libraries(HUML libipoptamplinterface.a)

