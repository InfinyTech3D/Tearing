##############################################################################
#                 - Copyright (C) 2020-Present InfinyTech3D -                #
#                                                                            #
#  This file is part of the Tearing plugin for the SOFA framework.           #
#                                                                            #
#  This file is dual-licensed:                                               #
#                                                                            #
#  1) Commercial License:                                                    #
#       This file may be used under the terms of a valid commercial license  #
#       agreement provided wih the software by InfinyTech3D.                 #
#                                                                            #
#  2) GNU General Public License (GPLv3) Usage                               #
#       Alternatively, this file may be used under the terms of the          #
#       GNU General Public License version 3 as published by the             #
#       Free Software Foundation: https://www.gnu.org/licenses/gpl-3.0.html  #
#                                                                            #
#  Contact: contact@infinytech3d.com                                         #
#  Further information: https://infinytech3d.com                             #
##############################################################################

cmake_minimum_required(VERSION 3.12)
project(Tearing_test VERSION 1.0)

find_package(Sofa.Testing REQUIRED)

set(SOURCE_FILES
    TearingEngine_test.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")

target_link_libraries(${PROJECT_NAME} Tearing Sofa.Testing)

add_definitions("-DSOFA_TEARING_TEST_SCENES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/../scenes/\"")

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
