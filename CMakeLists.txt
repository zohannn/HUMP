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
set(PROJECT_NAME_STR HUML)
project(${PROJECT_NAME_STR} C CXX)

SET(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

## Debug definition: ##
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions("-DDEBUG")
endif()

##############################################################################
# Settings
##############################################################################


## tb = ]0,1[,
## it constrols when the bounce posture is reached.
## 0.5 means at the middle of the movement
set(TB 0.5)
## AMPL software
# AMPL=0 use wine to run ampl for windows
# AMPL=1 use ampl for linux
set (AMPL 0)
set (AMPL_PATH /home/gianpaolo/AMPL)

# -- IPOPT Solver
set(IPOPT_HOME /home/gianpaolo/CoinIpopt)

## Tests
# TEST=0. do not run the tests
# TEST=1, run the tests
set(TEST 0)

## Documentation
# DOC=0. do not make the documentation
# DOC=1, make the documentation
set(DOC 1)

##############################################################################

# CoinIpopt
include_directories(
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/coin
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/coin/ThirdParty
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/extra

)

# sources
file(GLOB_RECURSE HPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp config/*.hpp)
file(GLOB_RECURSE CPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

# configuration files
configure_file (
  "${PROJECT_SOURCE_DIR}/config/config.hpp.in"
  "${PROJECT_SOURCE_DIR}/config/config.hpp"
)

# add the library
add_library(HUML STATIC ${HPP_SOURCES} ${CPP_SOURCES})


##############################################################################
# Testing
##############################################################################
if(TEST)
    add_subdirectory(test/lib/gtest-1.6.0)
    include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
    enable_testing ()

    ## Point tests
    set(PROJECT_TEST_NAME ${PROJECT_NAME_STR}_point_tests)
    file(GLOB TEST_CPP_FILES ${PROJECT_SOURCE_DIR}/test/src/point_tests.cpp)
    #file(GLOB TEST_HPP_FILES ${PROJECT_SOURCE_DIR}/test/include/*.hpp)
    #add_executable(${PROJECT_TEST_NAME} ${TEST_CPP_FILES} ${TEST_HPP_FILES})
    add_executable(${PROJECT_TEST_NAME} ${TEST_CPP_FILES})
    target_link_libraries(${PROJECT_TEST_NAME} gtest gtest_main)
    target_link_libraries(${PROJECT_TEST_NAME} HUML)
    add_test (NAME Tests_on_point COMMAND ${PROJECT_TEST_NAME})

endif()


##############################################################################
# Documenting
##############################################################################
if(DOC)
    find_package(Doxygen)
    if(NOT DOXYGEN_FOUND)
        message(WARNING "Doxygen not found, unable to generate documentation")
    else()
        add_custom_target(doc COMMAND ${DOXYGEN_EXECUTABLE} "doxyconfig/doxyfile.ini"
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            COMMENT "Building user's documentation into the doc directory ..."
        )
    endif()
endif()
