﻿cmake_minimum_required(VERSION 3.5.0)
project(AirSim)

add_subdirectory("rpclib_wrapper")
add_subdirectory("AirLib")
add_subdirectory("MavLinkCom")
if(BUILD_TESTS)
    add_subdirectory("AirLibUnitTests")
    add_subdirectory("HelloDrone")
    add_subdirectory("HelloSpawnedDrones")
    add_subdirectory("HelloCar")
    add_subdirectory("DroneShell")
    add_subdirectory("DroneServer")
endif()


